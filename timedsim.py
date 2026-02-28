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

NUM_NODES = 50
PDDL_FOLDER = "./pddl_files/"
get_environment().credits_stream = None

def generate_ground_truth(graph):
    """Generates random ground truth traps for the graph nodes."""
    gt = {}
    trap_types = ["trap_push", "trap_pic", "trap_animal", "clear", "clear"]
    for loc in graph.locations:
        gt[loc.name] = random.choice(trap_types)
    return gt

# ==========================================
#  VISUALIZATION ENGINE
# ==========================================
def create_mission_gif(graph, history, filename="mission_replay.gif"):
    print("\nGenerating mission animation... (This might take a few seconds)")
    
    fig, ax = plt.subplots(figsize=(12, 10))
    # Organic complex layout (k spaces the nodes out)
    pos = nx.spring_layout(graph.graph, seed=10, k=0.8) 

    legend_elements = [
        mpatches.Patch(color='lightgray', label='Unknown (Fog of War)'),
        mpatches.Patch(color='lightgreen', label='Clear / Safe'),
        mpatches.Patch(color='lightcoral', label='Trap: Push (Spot only)'),
        mpatches.Patch(color='plum', label='Trap: Pic (Rangers only)'),
        mpatches.Patch(color='gold', label='Trap: Animal (Rangers only)'),
        mpatches.Patch(facecolor='white', edgecolor='black', linewidth=3, label='Inspected Boundary')
    ]

    def update(frame_idx):
        ax.clear()
        state = history[frame_idx]
        
        # 1. Draw Edges
        nx.draw_networkx_edges(graph.graph, pos, ax=ax, edge_color="lightgray", style="dotted")
        traversable_edges = list(graph.traversable_edges)
        nx.draw_networkx_edges(graph.graph, pos, ax=ax, edgelist=traversable_edges, edge_color="green", width=2)

        # 2. Node colors based on fog of war
        node_colors = []
        edge_colors = []
        linewidths = []
        
        for node in graph.graph.nodes():
            is_inspected = node in state["inspected"]
            linewidths.append(3 if is_inspected else 1)
            edge_colors.append("black" if is_inspected else "gray")
            
            status = state["knowledge"].get(node, "unknown")
            if not is_inspected:
                node_colors.append("lightgray")
            elif status == "clear":
                node_colors.append("lightgreen")
            elif status == "trap_push":
                node_colors.append("lightcoral") 
            elif status == "trap_pic":
                node_colors.append("plum")       
            elif status == "trap_animal":
                node_colors.append("gold")       
            else:
                node_colors.append("white")
                
        # 3. Draw Nodes
        nx.draw_networkx_nodes(graph.graph, pos, ax=ax, node_color=node_colors, 
                               edgecolors=edge_colors, linewidths=linewidths, node_size=600)
        nx.draw_networkx_labels(graph.graph, pos, ax=ax, font_size=9, font_weight="bold")

        # 4. Draw Agents
        agent_colors = {"r_drone": "cyan", "r_spot": "orange", "caro": "purple", "bapt": "blue"}
        agent_offsets = {"r_drone": (0, 0.12), "r_spot": (0, -0.12), "caro": (-0.12, 0), "bapt": (0.12, 0)}
        
        for agent, loc in state["agents"].items():
            if loc in pos:
                x, y = pos[loc]
                dx, dy = agent_offsets[agent]
                ax.text(x + dx, y + dy, agent.split('_')[-1].upper(), 
                        color="black" if agent == "r_drone" else "white", 
                        fontsize=8, fontweight='bold',
                        bbox=dict(facecolor=agent_colors[agent], edgecolor='none', boxstyle='round,pad=0.2'))

        # 5. Draw Animal
        anim_loc = state["animal"]
        if anim_loc.startswith("carried_by_"):
            carrier = anim_loc.replace("carried_by_", "")
            carrier_loc = state["agents"].get(carrier)
            if carrier_loc in pos:
                x, y = pos[carrier_loc]
                ax.text(x, y + 0.20, "ANIMAL", color="black", fontsize=8, fontweight='bold',
                        bbox=dict(facecolor="yellow", edgecolor='black', boxstyle='round,pad=0.2'))
        else:
            if anim_loc in pos:
                x, y = pos[anim_loc]
                ax.text(x, y + 0.20, "ANIMAL", color="black", fontsize=8, fontweight='bold',
                        bbox=dict(facecolor="yellow", edgecolor='black', boxstyle='round,pad=0.2'))

        # 6. Title and Legend
        ax.set_title(f"Step {frame_idx + 1}/{len(history)}\nAction: {state['action']}", 
                     fontsize=12, fontweight="bold")
        ax.legend(handles=legend_elements, loc='lower center', ncol=3, bbox_to_anchor=(0.5, -0.15))
        ax.axis('off')

    anim = FuncAnimation(fig, update, frames=len(history), interval=600)
    anim.save(filename, writer=PillowWriter(fps=2.0))
    plt.close()
    print(f"Animation saved as '{filename}'!")

# ==========================================
#  CORE SIMULATION
# ==========================================
def run_mission(planner_name='fast-downward', graph=None, ground_truth=None, verbose=True, make_gif=True, gif_path=None):
    if verbose:
        print(f"\n--- STARTING FULL EXPLORATION MISSION ---")
        print(f"   Planner: {planner_name}")
    
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
    
    history = []
    def save_state(action_text):
        if make_gif:
            history.append({
                "agents": deepcopy(agents_state),
                "knowledge": deepcopy(knowledge),
                "inspected": deepcopy(inspected_nodes),
                "animal": animal_state,
                "action": action_text
            })
    
    save_state("Mission Start")

    try:
        planner = OneshotPlanner(name=planner_name)
    except Exception as e:
        if verbose: print(f"   [!] Error loading {planner_name}: {e}")
        return 0.0, 0, 0, False

    mission_complete = False
    step = 0
    total_compute_time = 0.0
    total_actions_executed = 0

    while not mission_complete:
        step += 1
        if verbose: print(f"\n[STEP {step}] Computing strategy...")
        
        up_problem = generate_classic_pddl(
            graph, 
            knowledge, 
            unique_id=f"{planner_name}_{step}", 
            agents_state=agents_state,
            animal_state=animal_state
        )
        
        start_time = time.time()
        result = planner.solve(up_problem)
        end_time = time.time()
        
        total_compute_time += (end_time - start_time)
        
        if result.status not in [PlanGenerationResultStatus.SOLVED_SATISFICING, PlanGenerationResultStatus.SOLVED_OPTIMALLY]:
            if verbose: print("   FATAL ERROR: Path is blocked.")
            break

        if not result.plan or not result.plan.actions:
            mission_complete = True
            break
            
        if verbose: print(f"   [EXEC] Executing planned sequence:")
        
        for action_instance in result.plan.actions:
            total_actions_executed += 1
            
            action_name = action_instance.action.name
            params = action_instance.actual_parameters
            action_text = f"{action_name}({', '.join(map(str, params))})"
            
            if verbose: print(f"      -> {action_text}")
            
            if "move" in action_name:
                agents_state[str(params[2])] = str(params[1])
            elif "disarm" in action_name:
                knowledge[str(params[0])] = "clear"
            elif "carry" in action_name:
                animal_state = f"carried_by_{str(params[1])}"
            elif "deliver" in action_name:
                animal_state = str(params[0])
            elif "inspect" in action_name:
                target_loc = str(params[0]) if action_name == "drone_inspect" else str(params[1])
                agent = str(params[1]) if action_name == "drone_inspect" else str(params[2])
                
                if verbose: print(f"   [📡 SCOUT] {agent} is inspecting {target_loc}...")
                real_state = ground_truth.get(target_loc, "clear")
                
                knowledge[target_loc] = real_state
                inspected_nodes.add(target_loc)
                
                if verbose: print(f"      👀 Revealed: {real_state} at {target_loc}")
                save_state(action_text)
                
                if real_state != "clear":
                    if verbose: print("   [ALERT] Obstacle found! Halting execution to replan.")
                    break 
                continue 

            save_state(action_text)

        if step > 250:
            if verbose: print("   [!] Safety catch: Exceeded 250 replanning steps.")
            break

    if mission_complete:
        if verbose: print("\nMISSION ACCOMPLISHED.")
        if make_gif:
            if gif_path is None: gif_path = f"mission_{planner_name}.gif"
            create_mission_gif(graph, history, filename=gif_path)
    else:
        if verbose: print("\nMISSION FAILED OR ABORTED.")

    planner.destroy()

    # RETURN 4 VALUES NOW
    return total_compute_time, step, total_actions_executed, mission_complete


if __name__ == "__main__":
    if not os.path.exists(PDDL_FOLDER): os.makedirs(PDDL_FOLDER)
    run_mission()