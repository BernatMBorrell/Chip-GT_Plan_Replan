import random
from unified_planning.shortcuts import *
from unified_planning.engines import PlanGenerationResultStatus

from graphgenerator import Graph
from domaingenerator import generate_classic_pddl

NUM_NODES = 10  # Manténlo en 10 para ver la prueba rápidamente
get_environment().credits_stream = None

def generate_ground_truth(graph):
    """Generates random traps on the graph nodes."""
    gt = {}
    trap_types = ["trap_push", "trap_pic", "trap_animal", "clear"]
    for loc in graph.locations:
        gt[loc.name] = random.choice(trap_types)
    return gt

def run_mission():
    print(f"\n--- 🚀 STARTING SCOUTING MISSION ON GRAPH TOPOLOGY ---")
    
    # 1. Create the physical world
    graph = Graph(num_nodes=NUM_NODES, seed=42, extra_edges=4, traversable_prob=0.5)
    ground_truth = generate_ground_truth(graph)
    
    # 2. Initial Beliefs
    knowledge = {l.name: "unknown" for l in graph.locations}
    agents_state = {"r_spot": "l_start", "r_drone": "l_start", "caro": "l_start", "bapt": "l_start"}
    
    planner = OneshotPlanner(name='fast-downward')
    mission_complete = False
    step = 0

    while not mission_complete:
        step += 1
        print(f"\n[STEP {step}] Agents: {agents_state}")

        print("   [🧠 PLAN] Computing strategy...")
        up_problem = generate_classic_pddl(graph, knowledge, unique_id=str(step), agents_state=agents_state)
        result = planner.solve(up_problem)
        
        if result.status != PlanGenerationResultStatus.SOLVED_SATISFICING:
            print("   ❌ FATAL ERROR: Path is blocked. Ground truth requires impossible actions.")
            break

        if not result.plan.actions:
            # Si el planificador devuelve un plan vacío pero el estado es SOLVED, 
            # significa que ¡ya hemos cumplido todas las metas!
            mission_complete = True
            break
            
        print(f"   [🚗 EXEC] Executing planned sequence:")
        for action_instance in result.plan.actions:
            action_name = action_instance.action.name
            params = action_instance.actual_parameters
            print(f"      -> {action_name}({', '.join(map(str, params))})")
            
            # --- MOVIMIENTO ---
            if "move" in action_name:
                agent = str(params[2]) 
                dest = str(params[1])
                agents_state[agent] = dest

            # --- DESARME ---
            elif "disarm" in action_name:
                loc = str(params[0]) 
                knowledge[loc] = "clear"

            # --- INSPECCIÓN (SCOUTING) ---
            elif "inspect" in action_name:
                agent = str(params[2])
                target_loc = str(params[1])
                
                print(f"   [📡 SCOUT] {agent} is inspecting {target_loc}...")
                real_state = ground_truth.get(target_loc, "clear")
                
                knowledge[target_loc] = real_state
                print(f"      👀 Revealed: {real_state} at {target_loc}")
                
                # Si no está limpio, invalidamos el plan
                if real_state != "clear":
                    print("   [⚠️ ALERT] Obstacle found! Halting execution to coordinate a disarm.")
                    break 

        if all(loc == "l_end" for loc in agents_state.values()):
            mission_complete = True
            break

    if mission_complete:
        print("\n🏆 ALL AGENTS REACHED L_END! MISSION ACCOMPLISHED.")
    else:
        print("\n💥 MISSION FAILED OR ABORTED.")

if __name__ == "__main__":
    run_mission()