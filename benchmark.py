import os
import multiprocessing
import matplotlib.pyplot as plt
import warnings
import numpy as np

# Mute the UP warnings
warnings.filterwarnings("ignore", module="unified_planning")

from graphgenerator import Graph
from timedsim import run_mission, generate_ground_truth

# --- CONFIGURATION ---
# Focusing only on the Numeric Planner for the "Lazy Ranger" study
planners_to_test = {
    "ENHSP (Lazy Rangers)": "enhsp"
}

complexity_lvls = [3, 4, 5, 7, 10, 15, 20, 30] 
SEEDS = [42, 63, 57, 100, 59, 19, 58, 29, 68]
TIME_LIMIT = 60 * 3 

def test_planner_mp(planner_engine, graph, ground_truth, lvl, seed, result_queue):
    try:
        # Note: Added agent_costs and swaps to the return catch
        time_taken, replan_steps, solution_steps, success, agent_costs, portfolio_swaps, full_replans = run_mission(
            planner_name=planner_engine, 
            graph=graph, 
            ground_truth=ground_truth, 
            verbose=False,        
            make_gif=False
        )
        # Calculate the cumulative mathematical cost: 1*Drone + 2*Spot + 4*Bapt + 6*Caro
        total_objective_cost = sum(agent_costs.values())
        
        status = "SUCCESS" if success else "FAILED"
        result_queue.put((time_taken, replan_steps, solution_steps, status, total_objective_cost))
    except Exception as e:
        result_queue.put((0.0, 0, 0, f"ERROR: {e}", 0))

def benchmark():
    figures_dir = "Figures/benchmark_results"
    solutions_dir = "solutions"
    report_path = f"{figures_dir}/benchmark_report.txt"
    os.makedirs(figures_dir, exist_ok=True)
    os.makedirs(solutions_dir, exist_ok=True)

    results = {p: {lvl: {} for lvl in complexity_lvls} for p in planners_to_test}
    blocked = {name: False for name in planners_to_test}

    report_lines = []
    def log(msg):
        print(msg)
        report_lines.append(msg)

    log("--- STARTING NUMERIC CUMULATIVE COST BENCHMARK ---")
    
    for lvl in complexity_lvls:
        for seed in SEEDS:
            log(f"\n LEVEL: {lvl} NODES | SEED: {seed}")
            test_graph = Graph(num_nodes=lvl, seed=seed, extra_edges=int(lvl * 1.5), traversable_prob=0.7)
            test_ground_truth = generate_ground_truth(test_graph)

            for display_name, engine_name in planners_to_test.items():
                if blocked[display_name]:
                    results[display_name][lvl][seed] = (TIME_LIMIT, 0, 0, "TIMEOUT", 0)
                    continue

                result_queue = multiprocessing.Queue()
                p = multiprocessing.Process(
                    target=test_planner_mp,
                    args=(engine_name, test_graph, test_ground_truth, lvl, seed, result_queue)
                )
                p.start()
                p.join(timeout=TIME_LIMIT)

                if p.is_alive():
                    log(f"      [TIMEOUT] {display_name}")
                    p.terminate()
                    p.join()
                    blocked[display_name] = True
                    results[display_name][lvl][seed] = (TIME_LIMIT, 0, 0, "TIMEOUT", 0)
                else:
                    if not result_queue.empty():
                        res = result_queue.get()
                    else:
                        res = (0.0, 0, 0, "ERROR", 0)
                    
                    log(f"      [{res[3]}] Time: {res[0]:.2f}s | Cost: {res[4]}")
                    results[display_name][lvl][seed] = res

    # ==========================================
    # DATA AGGREGATION
    # ==========================================
    avg_times = []
    avg_costs = []
    avg_actions = []

    # Since we only have one planner (ENHSP), we extract its specific data
    planner_name = "ENHSP (Lazy Rangers)"
    for lvl in complexity_lvls:
        lvl_data = results[planner_name][lvl]
        successes = [d for d in lvl_data.values() if d[3] == "SUCCESS"]
        
        if successes:
            avg_times.append(sum(d[0] for d in successes) / len(successes))
            avg_costs.append(sum(d[4] for d in successes) / len(successes))
            avg_actions.append(sum(d[2] for d in successes) / len(successes))
        else:
            avg_times.append(0); avg_costs.append(0); avg_actions.append(0)

    # ==========================================
    # NEW PAPER PLOT: CUMULATIVE COST vs ACTIONS
    # ==========================================
    fig, ax1 = plt.subplots(figsize=(10, 6))

    color_cost = 'tab:orange'
    ax1.set_xlabel('Map Complexity (Number of Locations)', fontweight='bold')
    ax1.set_ylabel('Average Cumulative Mission Cost', color=color_cost, fontweight='bold')
    ax1.plot(complexity_lvls, avg_costs, color=color_cost, marker='D', linewidth=3, markersize=10, label='Objective Cost (Weighted)')
    ax1.tick_params(axis='y', labelcolor=color_cost)
    ax1.grid(True, linestyle='--', alpha=0.6)

    ax2 = ax1.twinx()
    color_actions = 'tab:blue'
    ax2.set_ylabel('Total Actions Executed', color=color_actions, fontweight='bold')
    ax2.plot(complexity_lvls, avg_actions, color=color_actions, marker='o', linestyle=':', linewidth=2, label='Number of Actions')
    ax2.tick_params(axis='y', labelcolor=color_actions)

    plt.title('Scaling Analysis: Weighted Mission Cost vs. Raw Activity', fontsize=14, fontweight='bold')
    fig.tight_layout()
    
    plot_path = f"{figures_dir}/cumulative_cost_analysis.png"
    plt.savefig(plot_path)
    log(f"\n[ANALYSIS] Plot saved to {plot_path}")
    
    with open(report_path, "w") as f:
        f.write("\n".join(report_lines))

if __name__ == "__main__":
    benchmark()