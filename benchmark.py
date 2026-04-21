import os
import multiprocessing
import matplotlib.pyplot as plt
import warnings
import numpy as np
import time
from datetime import datetime

# Mute unified_planning warnings
warnings.filterwarnings("ignore", module="unified_planning")

from graphgenerator import Graph
from timedsim import run_mission, generate_ground_truth

# --- CONFIGURATION ---
planners_to_test = {
    "ENHSP (Lazy Rangers)": "enhsp"
}

complexity_lvls = [3, 4, 5, 7, 10, 15, 20, 30, 40, 50] 
SEEDS = [42, 63, 57, 100, 59, 19, 58, 29, 68]
TIME_LIMIT = 60 * 10

# Asymmetric displacement times (seconds)
DISPLACEMENT_TIMES = {
    "aerial": 10.0, 
    "ground": 10.0
}

# Parallelization control
MAX_CONCURRENT_TASKS = 5

def test_planner_mp(planner_engine, graph, ground_truth, lvl, seed, result_queue):
    try:
        (time_taken, replan_steps, solution_steps, success, agent_costs, 
         portfolio_swaps, full_replans, init_latency, wait_latency) = run_mission(
            planner_name=planner_engine, 
            graph=graph, 
            ground_truth=ground_truth, 
            verbose=False,        
            make_gif=False,
            displacement_times=DISPLACEMENT_TIMES
        )
        total_objective_cost = sum(agent_costs.values())
        status = "SUCCESS" if success else "FAILED"
        result_queue.put((time_taken, replan_steps, solution_steps, status, total_objective_cost, init_latency, wait_latency))
    except Exception as e:
        result_queue.put((0.0, 0, 0, f"ERROR: {e}", 0, 0.0, 0.0))

def benchmark():
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    figures_dir = f"Figures/benchmark_results_{timestamp}"
    solutions_dir = f"solutions_{timestamp}"
    report_path = f"{figures_dir}/benchmark_report.txt"
    
    os.makedirs(figures_dir, exist_ok=True)
    os.makedirs(solutions_dir, exist_ok=True)

    results = {p: {lvl: {} for lvl in complexity_lvls} for p in planners_to_test}
    report_lines = []

    def log(msg):
        print(msg)
        report_lines.append(msg)

    log(f"--- STARTING MULTIPROCESSING BENCHMARK (ID: {timestamp}) ---")
    log(f"Using asymmetric displacements: Drones {DISPLACEMENT_TIMES['aerial']}s | Ground {DISPLACEMENT_TIMES['ground']}s")
    log(f"Running up to {MAX_CONCURRENT_TASKS} simulations in parallel...\n")
    
    # 1. Initialize task queue
    tasks_to_run = []
    for lvl in complexity_lvls:
        for seed in SEEDS:
            for display_name, engine_name in planners_to_test.items():
                tasks_to_run.append((display_name, engine_name, lvl, seed))
    
    total_tasks = len(tasks_to_run)
    completed_tasks = 0
    active_tasks = []

    # 2. Main multiprocessing control loop
    while tasks_to_run or active_tasks:
        for task in active_tasks[:]:
            p = task['process']
            q = task['queue']
            start_time = task['start_time']
            display_name, engine_name, lvl, seed = task['info']
            
            if not p.is_alive():
                # Process completed normally
                res = q.get() if not q.empty() else (0.0, 0, 0, "ERROR", 0, 0.0, 0.0)
                completed_tasks += 1
                log(f"[{completed_tasks:02d}/{total_tasks}] [{res[3]}] Lvl: {lvl:2d} | Seed: {seed:3d} | CPU: {res[0]:6.2f}s | Wait: {res[6]:6.2f}s")
                results[display_name][lvl][seed] = res
                active_tasks.remove(task)
                
            elif time.time() - start_time > TIME_LIMIT:
                # Process exceeded time limit; terminate
                p.terminate()
                p.join()
                completed_tasks += 1
                log(f"[{completed_tasks:02d}/{total_tasks}] [TIMEOUT] Lvl: {lvl:2d} | Seed: {seed:3d} | Exceeded {TIME_LIMIT}s")
                results[display_name][lvl][seed] = (TIME_LIMIT, 0, 0, "TIMEOUT", 0, 0.0, 0.0)
                active_tasks.remove(task)
                
        # Start new tasks if pool capacity allows
        while len(active_tasks) < MAX_CONCURRENT_TASKS and tasks_to_run:
            display_name, engine_name, lvl, seed = tasks_to_run.pop(0)
            
            test_graph = Graph(num_nodes=lvl, seed=seed, extra_edges=int(lvl * 1.5), traversable_prob=0.7)
            test_ground_truth = generate_ground_truth(test_graph)
            
            q = multiprocessing.Queue()
            p = multiprocessing.Process(
                target=test_planner_mp, args=(engine_name, test_graph, test_ground_truth, lvl, seed, q)
            )
            p.start()
            active_tasks.append({
                'process': p,
                'queue': q,
                'info': (display_name, engine_name, lvl, seed),
                'start_time': time.time()
            })
            
        time.sleep(1.0) # Prevent CPU thrashing

    log("\nAll simulations completed! Aggregating data and generating plots...")

    # ==========================================
    # DATA AGGREGATION
    # ==========================================
    planner_name = "ENHSP (Lazy Rangers)"
    avg_cpu_times, avg_wait_times, avg_init_latency, avg_costs, avg_actions, success_rates = [], [], [], [], [], []

    for idx, lvl in enumerate(complexity_lvls):
        lvl_data = results[planner_name][lvl]
        successes = [d for d in lvl_data.values() if d[3] == "SUCCESS"]
        s_rate = (len(successes) / len(SEEDS)) * 100
        success_rates.append(s_rate)
        
        if successes:
            m_cpu_time = sum(d[0] for d in successes) / len(successes)
            m_action = sum(d[2] for d in successes) / len(successes)
            m_cost = sum(d[4] for d in successes) / len(successes)
            m_init = sum(d[5] for d in successes) / len(successes)
            m_wait = sum(d[6] for d in successes) / len(successes)
            
            avg_cpu_times.append(m_cpu_time); avg_costs.append(m_cost); avg_actions.append(m_action)
            avg_init_latency.append(m_init); avg_wait_times.append(m_wait)
        else:
            avg_cpu_times.append(0); avg_costs.append(0); avg_actions.append(0)
            avg_init_latency.append(0); avg_wait_times.append(0)

    # ==========================================
    # FINAL SCALING LINE PLOT
    # ==========================================
    fig, ax1 = plt.subplots(figsize=(14, 7))

    x_indices = np.arange(len(complexity_lvls))
    width = 0.25 

    # Academic color palette
    color_cpu = '#4C72B0'   
    color_init = '#55A868'  
    color_wait = '#8172B3'  
    color_cost = '#202020'  

    ax1.set_xlabel('Map Complexity (Locations)', fontweight='bold')
    ax1.set_ylabel('Time (Seconds)', color='black', fontweight='bold')
    
    bars1 = ax1.bar(x_indices - width, avg_cpu_times, width, color=color_cpu, edgecolor='white', label='Total CPU Compute Time')
    for i, (bar, value) in enumerate(zip(bars1, avg_cpu_times)):
        height = bar.get_height()
        ax1.text(bar.get_x() + bar.get_width()/2., height,
                f'{value:.1f}s', ha='center', va='bottom', fontsize=8, color=color_cpu, fontweight='bold')
    
    bars2 = ax1.bar(x_indices, avg_init_latency, width, color=color_init, edgecolor='white', label='Initial Mission Latency')
    for bar, value in zip(bars2, avg_init_latency):
        height = bar.get_height()
        ax1.text(bar.get_x() + bar.get_width()/2., height,
                f'{value:.1f}s', ha='center', va='bottom', fontsize=8, color=color_init, fontweight='bold')
    
    bars3 = ax1.bar(x_indices + width, avg_wait_times, width, color=color_wait, edgecolor='white', label='Robot Replanning Wait (Latency)')
    for bar, value in zip(bars3, avg_wait_times):
        height = bar.get_height()
        ax1.text(bar.get_x() + bar.get_width()/2., height,
                f'{value:.1f}s', ha='center', va='bottom', fontsize=8, color=color_wait, fontweight='bold')
    
    ax1.set_xticks(x_indices)
    ax1.set_xticklabels([str(l) for l in complexity_lvls])
    ax1.grid(True, axis='y', linestyle='--', alpha=0.4)

    ax2 = ax1.twinx()
    ax2.set_ylabel('Cumulative Mission Cost', color=color_cost, fontweight='bold')
    line1, = ax2.plot(x_indices, avg_costs, color=color_cost, marker='D', linewidth=2.5, markersize=7, label='Objective Cost')
    ax2.tick_params(axis='y', labelcolor=color_cost)

    plt.title('Portfolio Planning: Computational Effort vs Real-World Latencies', fontsize=14, fontweight='bold')
    
    lines = [bars1, bars2, bars3, line1]
    labels = [l.get_label() for l in lines]
    ax1.legend(lines, labels, loc='upper left', bbox_to_anchor=(0.02, 0.98), framealpha=0.9)

    fig.tight_layout()
    plt.savefig(f"{figures_dir}/latency_vs_cost_analysis.png", dpi=300) 
    plt.close()

    # ==========================================
    # SUCCESS RATE PLOT
    # ==========================================
    plt.figure(figsize=(10, 6))
    
    sr_colors = ['#55A868' if sr >= 80 else '#DD8452' if sr >= 50 else '#C44E52' for sr in success_rates]
    bars = plt.bar(complexity_lvls, success_rates, color=sr_colors, alpha=0.85, edgecolor='black', linewidth=1.2)
    
    for x, sr in zip(complexity_lvls, success_rates):
        plt.text(x, sr + 2, f'{sr:.0f}%', ha='center', va='bottom', fontweight='bold', fontsize=10)
    
    plt.axhline(y=100, color='#55A868', linestyle='--', alpha=0.7, label='100% Success')
    plt.axhline(y=50, color='#DD8452', linestyle='--', alpha=0.7, label='50% Success')
    
    plt.title("Success Rate vs Map Complexity", fontsize=14, fontweight='bold')
    plt.xlabel("Number of Locations (N)", fontsize=12)
    plt.ylabel("Success Rate (%)", fontsize=12)
    plt.ylim(0, 110)
    plt.legend(fontsize=10)
    plt.grid(axis='y', linestyle='--', alpha=0.4)
    plt.tight_layout()
    plt.savefig(f"{figures_dir}/success_rate.png", dpi=300)
    plt.close()
    
    with open(report_path, "w") as f: f.write("\n".join(report_lines))
    print(f"Benchmark Complete! All plots saved in {figures_dir}")

if __name__ == "__main__":
    benchmark()