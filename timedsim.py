import os
import random
import time
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import networkx as nx
from matplotlib.animation import FuncAnimation, PillowWriter
from copy import deepcopy

from unified_planning.shortcuts import *
from unified_planning.engines import PlanGenerationResultStatus

from graphgenerator import Graph
from domaingenerator import generate_classic_pddl

NUM_NODES = 15
PDDL_FOLDER = "./pddl_files/"
get_environment().credits_stream = None

def generate_ground_truth(graph):
    gt = {}
    trap_types = ["trap_push", "trap_pic", "trap_animal", "clear", "clear"]
    for loc in graph.locations:
        gt[loc.name] = random.choice(trap_types)
    return gt

def create_mission_gif(graph, history, filename="mission_replay.gif"):
    # (Mismo código de visualización que tenías, lo omito para no alargar el bloque, 
    # asume que es exactamente igual al tuyo)
    print(f"Animation saved as '{filename}'!")

# ==========================================
#  CORE SIMULATION
# ==========================================
def run_mission(planner_name='fast-downward', graph=None, ground_truth=None, verbose=True, make_gif=True, gif_path=None, save_pddl=True, displacement_times=None):
    if displacement_times is None:
        displacement_times = {"aerial": 10.0, "ground": 30.0}

    if verbose:
        print(f"\n--- STARTING FULL EXPLORATION MISSION ---")
        print(f"   Planner: {planner_name} | Displacement Time: {displacement_time}s")
    
    if graph is None:
        graph = Graph(num_nodes=NUM_NODES, seed=42, extra_edges=8, traversable_prob=0.7)
    if ground_truth is None:
        ground_truth = generate_ground_truth(graph)
    
    knowledge = {l.name: "unknown" for l in graph.locations}
    knowledge["l_start"] = "clear"
    knowledge["l_end"] = "clear"
    
    agents_state = {"r_spot": "l_start", "r_drone": "l_start", "caro": "l_start", "bapt": "l_start"}
    inspected_nodes = {"l_start", "l_end"}
    animal_state = "l_02" if len(graph.locations) >= 2 else "l_01"
    
    COST_WEIGHTS = {"r_drone": 1, "r_spot": 2, "bapt": 4, "caro": 6}
    agent_costs = {"r_drone": 0, "r_spot": 0, "caro": 0, "bapt": 0}
    
    # NUEVAS MÉTRICAS DE LATENCIA
    portfolio_swaps = 0
    full_replans = 0
    initial_latency = 0.0
    total_wait_time = 0.0
    is_first_plan = True
    
    history = []
    def save_state(action_text):
        if make_gif:
            history.append({
                "agents": deepcopy(agents_state),
                "knowledge": deepcopy(knowledge),
                "inspected": deepcopy(inspected_nodes),
                "animal": animal_state,
                "action": action_text,
                "costs": deepcopy(agent_costs)
            })
    
    save_state("Mission Start")

    try:
        planner = OneshotPlanner(name=planner_name)
    except Exception as e:
        if verbose: print(f"   [!] Error loading {planner_name}: {e}")
        return 0.0, 0, 0, False, agent_costs, 0, 0, 0.0, 0.0

    mission_complete = False
    step = 0
    total_compute_time = 0.0
    total_actions_executed = 0

    portfolio = {}
    precomputed_plan_ready = False
    real_state_discovered = ""
    current_plan = None

    while not mission_complete:
        step += 1
        
        # --- 1. PLAN SELECTION ---
        if precomputed_plan_ready and real_state_discovered in portfolio:
            if verbose: print(f"\n[STEP {step}] SWAP: Hypothesis match! Using precomputed plan for '{real_state_discovered}'.")
            current_plan = portfolio[real_state_discovered]
            precomputed_plan_ready = False 
        else:
            if verbose: print(f"\n[STEP {step}] Computing strategy...")
            up_problem = generate_classic_pddl(graph, knowledge, unique_id=f"{planner_name}_{step}", agents_state=agents_state, animal_state=animal_state)
            
            start_time = time.time()
            result = planner.solve(up_problem)
            compute_time = time.time() - start_time
            
            total_compute_time += compute_time
            
            # GESTIÓN DE LATENCIA DE REPLANIFICACIÓN CLÁSICA
            if is_first_plan:
                initial_latency = compute_time
                is_first_plan = False
            else:
                # Si no es el primer plan y no usamos portfolio, el robot estuvo parado esperando TODO este tiempo
                total_wait_time += compute_time
                full_replans += 1

            if result.status not in [PlanGenerationResultStatus.SOLVED_SATISFICING, PlanGenerationResultStatus.SOLVED_OPTIMALLY]:
                if verbose: print("   FATAL ERROR: Path is blocked.")
                break
                
            current_plan = result.plan

        if not current_plan or not current_plan.actions:
            mission_complete = True
            break
            
        if verbose: print(f"   [EXEC] Executing planned sequence:")
        
        # --- 2. EXECUTION LOOP ---
        for action_instance in current_plan.actions:
            total_actions_executed += 1
            action_name = action_instance.action.name
            params = action_instance.actual_parameters
            action_text = f"{action_name}({', '.join(map(str, params))})"
            
            if verbose: print(f"      -> {action_text}")
            
            for param in params:
                param_str = str(param)
                if param_str in agent_costs:
                    agent_costs[param_str] += COST_WEIGHTS[param_str]
                    break
            
            if "move" in action_name:
                agents_state[str(params[2])] = str(params[1])
            elif "disarm" in action_name:
                knowledge[str(params[0])] = "clear"
            elif "carry" in action_name:
                animal_state = f"carried_by_{str(params[1])}"
            elif "deliver" in action_name:
                animal_state = str(params[0])
            elif "inspect" in action_name:
                target_loc = str(params[0]) 
                agent = str(params[1]) 
                
                # PRECOMPUTATION PHASE (The Portfolio)
                if verbose: print(f"   [BACKGROUND] Precomputing hypotheses during {displacement_time}s displacement...")
                portfolio = {} 
                possible_states = ["trap_push", "trap_pic", "trap_animal"] 
                
                # Track cumulative time to know exactly when each plan finished
                cumulative_bg_time = 0.0
                portfolio_completion_times = {}
                
                for hypothesis in possible_states:
                    hypo_knowledge = deepcopy(knowledge)
                    hypo_knowledge[target_loc] = hypothesis
                    hypo_problem = generate_classic_pddl(graph, hypo_knowledge, unique_id=f"hypo_{hypothesis}", agents_state=agents_state, animal_state=animal_state)
                    
                    h_start = time.time()
                    hypo_result = planner.solve(hypo_problem)
                    h_time = time.time() - h_start
                    
                    cumulative_bg_time += h_time
                    if hypo_result and hypo_result.plan:
                        portfolio[hypothesis] = hypo_result.plan
                        portfolio_completion_times[hypothesis] = cumulative_bg_time
                
                total_compute_time += cumulative_bg_time 
                
                real_state = ground_truth.get(target_loc, "clear")
                knowledge[target_loc] = real_state
                inspected_nodes.add(target_loc)
                
                if verbose: print(f"       Revealed: {real_state} at {target_loc}")
                
                if real_state != "clear":
                    if real_state in portfolio:
                        precomputed_plan_ready = True
                        real_state_discovered = real_state
                        portfolio_swaps += 1
                        
                        # CÁLCULO MÁGICO DE ESPERA (Replanning Latency)
                        # Si el plan tardó 1s y el displacement es 2s, wait_time = 0
                        # Si el plan tardó 3s y el displacement es 2s, wait_time = 1
                        time_to_ready = portfolio_completion_times[real_state]
                        current_displacement = displacement_times["aerial"] if "drone" in agent or "air" in agent else displacement_times["ground"]

                        wait_time = max(0.0, time_to_ready - current_displacement)
                        total_wait_time += wait_time
                        
                        if verbose: print(f"   [LATENCY] Plan for '{real_state}' ready in {time_to_ready:.2f}s. Robot waited: {wait_time:.2f}s")
                    else:
                        precomputed_plan_ready = False
                        if verbose: print("   [ALERT] Obstacle found! Forcing classic replan from scratch.")
                    break 
                continue

            save_state(action_text)

        if step > 250:
            break

    planner.destroy()

    return total_compute_time, step, total_actions_executed, mission_complete, agent_costs, portfolio_swaps, full_replans, initial_latency, total_wait_time

if __name__ == "__main__":
    if not os.path.exists(PDDL_FOLDER): os.makedirs(PDDL_FOLDER)
    import warnings
    warnings.filterwarnings("ignore", module="unified_planning")
    run_mission(planner_name='enhsp')