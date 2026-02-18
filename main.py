import os
import itertools
import time
from unified_planning.shortcuts import *
from unified_planning.engines import PlanGenerationResultStatus

# Import your generator
from domaingenerator import generate_classic_pddl

# --- CONFIGURATION ---
NUM_NODES = 50          
CACHE_SIZE = 5          
PDDL_FOLDER = "./pddl_files/"
TRAP_TYPES = ["clear", "trap_push", "trap_animal", "trap_pic"]

plan_database = {}

# ---------------------------------------------------------
# 1. HELPERS: REALITY SIMULATION
# ---------------------------------------------------------
def get_real_world_state(node_index):
    if node_index > NUM_NODES: return "clear"
    if node_index % 3 == 1: return "trap_push"
    elif node_index % 3 == 2: return "trap_animal"
    else: return "trap_pic"

# ---------------------------------------------------------
# 2. OFFLINE PHASE: GENERATE DATABASE
# ---------------------------------------------------------
def build_offline_database():
    print(f"\n--- 🏗️ BUILDING DATABASE (First {CACHE_SIZE} nodes) ---")
    
    combinations = list(itertools.product(TRAP_TYPES, repeat=CACHE_SIZE))
    print(f"Total combinations to calculate: {len(combinations)}")
    
    # Use Fast Downward (ensure it's installed via pip install unified-planning[fast-downward])
    with OneshotPlanner(name='fast-downward') as planner:
        for i, combo in enumerate(combinations):
            
            knowledge = {f"l_{j+1:02d}": state for j, state in enumerate(combo)}
            
            # --- MODIFIED: Get Object Directly (No File I/O) ---
            # passing 'i' as unique_id for the problem name
            problem = generate_classic_pddl(10, PDDL_FOLDER, knowledge=knowledge, unique_id=i)
            
            # Solve directly in memory
            result = planner.solve(problem)
            
            if result.status == PlanGenerationResultStatus.SOLVED_SATISFICING:
                plan_database[combo] = result.plan
            else:
                plan_database[combo] = None
            
            if i % 100 == 0: print(f"Processing {i}/{len(combinations)}...")

    print(f"✅ Database completed. {len(plan_database)} cached entries.\n")


# ---------------------------------------------------------
# 3. ONLINE PHASE: PLAN-REPLAN LOOP
# ---------------------------------------------------------
def run_mission():
    print(f"--- 🚀 STARTING MISSION (Total Nodes: {NUM_NODES}) ---")
    
    knowledge = {f"l_{i:02d}": "unknown" for i in range(1, NUM_NODES + 1)}
    agents_state = {
        "r_spot": "l_start",
        "r_drone": "l_start",
        "caro": "l_start",
        "bapt": "l_start"
    }

    current_spot_node = 0 
    planner = OneshotPlanner(name='fast-downward')
    steps = 0

    while current_spot_node < NUM_NODES:
        steps += 1
        next_node_idx = current_spot_node + 1
        next_loc_name = f"l_{next_node_idx:02d}"
        
        print(f"\n[Step {steps}] Agents Positions: {agents_state}")
        print(f"   Facing (Spot): {next_loc_name}")

        # --- A. SENSING ---
        real_situation = get_real_world_state(next_node_idx)
        if knowledge[next_loc_name] != real_situation:
            print(f"   👀 SENSOR: Discovered '{real_situation}' at {next_loc_name}")
            knowledge[next_loc_name] = real_situation
        
        # --- B. PLANNING ---
        plan_to_execute = None
        used_db = False

        # STRATEGY 1: DATABASE
        if next_node_idx <= CACHE_SIZE and agents_state["r_spot"] == f"l_{current_spot_node:02d}" if current_spot_node > 0 else "l_start":
            key_list = []
            for k in range(1, CACHE_SIZE + 1):
                state = knowledge[f"l_{k:02d}"]
                if state == "unknown": state = "clear" 
                key_list.append(state)
            
            db_key = tuple(key_list)
            
            if db_key in plan_database and plan_database[db_key] is not None:
                plan_to_execute = plan_database[db_key]
                used_db = True
                print("   💾 STRATEGY: DATABASE (Cache Hit)")
            else:
                print("   ⚠️ STRATEGY: DATABASE MISS")

        # STRATEGY 2: FF (ONLINE)
        if not used_db:
            print("   🧠 STRATEGY: FF (Online Replanning)")

            problem = generate_classic_pddl(
                NUM_NODES, 
                PDDL_FOLDER, 
                knowledge=knowledge, 
                unique_id="replan",
                agents_state=agents_state
            )
            
            result = planner.solve(problem)
            if result.status == PlanGenerationResultStatus.SOLVED_SATISFICING:
                plan_to_execute = result.plan
            else:
                print("   ❌ ERROR: FF found no solution. Mission Failed.")
                break

        # --- C. EXECUTION ---
        if plan_to_execute:
            actions = plan_to_execute.actions
            if not actions:
                print("   🏁 Empty plan. Have we arrived?")
                break
                
            first_action = actions[0]
            print(f"   ⚡ EXECUTING: {first_action}")
            
            action_name = first_action.action.name
            params = first_action.actual_parameters

        # Lógica para actualizar el diccionario
            if "move" in action_name:
                origin_obj = params[0]
                dest_obj = params[1]
                agent_obj = params[2]
                
                agent_name = str(agent_obj)
                dest_name = str(dest_obj)
                
                agents_state[agent_name] = dest_name
                print(f"   🔄 UPDATE: {agent_name} moved to {dest_name}")
                
                if agent_name == "r_spot":
                    if dest_name != "l_start":
                         try:
                            new_idx = int(dest_name.split("_")[1])
                            current_spot_node = new_idx
                         except:
                            pass  

                if get_real_world_state(current_spot_node + 1).startswith("trap"):
                     knowledge[f"l_{current_spot_node+1:02d}"] = get_real_world_state(current_spot_node + 1)

            elif "disarm" in action_name:
                target_loc = str(params[1])
                print(f"   🛠️ Trap at {target_loc} disarmed.")
                knowledge[target_loc] = "clear"

        else:
            break

    print("\n🏆 MISSION COMPLETED!")

if __name__ == "__main__":
    if not os.path.exists(PDDL_FOLDER): os.makedirs(PDDL_FOLDER)
    build_offline_database()
    run_mission()