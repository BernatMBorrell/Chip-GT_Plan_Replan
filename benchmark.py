import os
import multiprocessing
import matplotlib.pyplot as plt
import warnings

# Mute the UP warnings
warnings.filterwarnings("ignore", module="unified_planning")

from graphgenerator import Graph
from timedsim import run_mission, generate_ground_truth

# --- CONFIGURATION ---
planners_classical = {
    "Fast Downward": "fast-downward",
    "ENHSP": "enhsp",
    "LPG": "lpg"
}

complexity_lvls = [3, 4, 5, 7, 10, 15, 20, 30] 
SEEDS = [42, 63, 57, 100, 59, 19, 58, 29, 68]
TIME_LIMIT = 60 * 3 

def test_planner_mp(planner_engine, graph, ground_truth, lvl, seed, result_queue):
    try:
        gif_filename = f"solutions/{planner_engine}_lvl_{lvl}_seed_{seed}.gif"
        
        time_taken, replan_steps, solution_steps, success = run_mission(
            planner_name=planner_engine, 
            graph=graph, 
            ground_truth=ground_truth, 
            verbose=False,        
            make_gif=False,
            gif_path=gif_filename 
        )
        status = "SUCCESS" if success else "FAILED"
        result_queue.put((time_taken, replan_steps, solution_steps, status))
    except Exception as e:
        result_queue.put((0.0, 0, 0, f"ERROR: {e}"))

def benchmark():
    figures_dir = "Figures/benchmark_results"
    solutions_dir = "solutions"
    report_path = f"{figures_dir}/benchmark_report.txt"
    os.makedirs(figures_dir, exist_ok=True)
    os.makedirs(solutions_dir, exist_ok=True)

    # Data structure: results[planner][lvl][seed] = (time, r_steps, s_steps, status)
    results = {p: {lvl: {} for lvl in complexity_lvls} for p in planners_classical}
    blocked = {name: False for name in planners_classical}

    report_lines = []
    def log(msg):
        print(msg)
        report_lines.append(msg)

    log("---  STARTING ROBUST MULTI-SEED BENCHMARK ---")
    log(f"Planners: {list(planners_classical.keys())}")
    log(f"Complexity Levels: {complexity_lvls}")
    log(f"Seeds per level: {SEEDS}")
    log("==================================================\n")

    for lvl in complexity_lvls:
        for seed in SEEDS:
            log(f"\n GENERATING MAP - COMPLEXITY: {lvl} NODES | SEED: {seed}")
            
            test_graph = Graph(num_nodes=lvl, seed=seed, extra_edges=int(lvl * 1.5), traversable_prob=0.7)
            test_ground_truth = generate_ground_truth(test_graph)

            for display_name, engine_name in planners_classical.items():
                if blocked[display_name]:
                    log(f"   Skipping {display_name} (Blocked from previous timeout).")
                    results[display_name][lvl][seed] = (TIME_LIMIT, 0, 0, "TIMEOUT")
                    continue

                log(f"  Running {display_name}...")

                result_queue = multiprocessing.Queue()
                p = multiprocessing.Process(
                    target=test_planner_mp,
                    args=(engine_name, test_graph, test_ground_truth, lvl, seed, result_queue)
                )
                p.start()
                p.join(timeout=TIME_LIMIT)

                if p.is_alive():
                    log(f"      [TIMEOUT] {display_name} exceeded the {TIME_LIMIT/60:.1f} min limit.")
                    p.terminate()
                    p.join()
                    blocked[display_name] = True
                    results[display_name][lvl][seed] = (TIME_LIMIT, 0, 0, "TIMEOUT")
                else:
                    if not result_queue.empty():
                        elapsed_time, r_steps, s_steps, status = result_queue.get()
                    else:
                        elapsed_time, r_steps, s_steps, status = (0.0, 0, 0, "ERROR")
                    
                    log(f"      [{status}] Time: {elapsed_time:.3f}s | Replans: {r_steps} | Actions Executed: {s_steps}")
                    results[display_name][lvl][seed] = (elapsed_time, r_steps, s_steps, status)

    # ==========================================
    # CALCULATE AVERAGES
    # ==========================================
    log("\n\n==================================================")
    log("AVERAGED BENCHMARK SUMMARY")
    log("==================================================")

    avg_times = {p: [] for p in planners_classical}
    avg_r_steps = {p: [] for p in planners_classical}
    avg_s_steps = {p: [] for p in planners_classical}
    success_rates = {p: [] for p in planners_classical}

    for name in planners_classical:
        log(f"\n{name}:")
        for lvl in complexity_lvls:
            lvl_results = results[name][lvl]
            
            # Filter only successful runs for step averages to prevent skewed data
            successes = [data for data in lvl_results.values() if data[3] == "SUCCESS"]
            num_success = len(successes)
            total_runs = len(SEEDS)
            s_rate = (num_success / total_runs) * 100
            
            # Time average includes timeouts (capped at TIME_LIMIT) to penalize failing planners
            mean_time = sum(data[0] for data in lvl_results.values()) / total_runs
            
            # Step averages only count if it actually found a solution
            mean_r_steps = sum(data[1] for data in successes) / num_success if num_success > 0 else 0
            mean_s_steps = sum(data[2] for data in successes) / num_success if num_success > 0 else 0

            avg_times[name].append(mean_time)
            avg_r_steps[name].append(mean_r_steps)
            avg_s_steps[name].append(mean_s_steps)
            success_rates[name].append(s_rate)

            log(f"  Level {lvl:02d}: {mean_time:.3f}s avg | {mean_r_steps:.1f} avg replans | {mean_s_steps:.1f} avg actions | Success Rate: {s_rate:.0f}% ({num_success}/{total_runs})")

    # ==========================================
    # SAVE TEXT REPORT
    # ==========================================
    with open(report_path, "w") as f:
        f.write("\n".join(report_lines))
    print(f"\nFull detailed report saved to '{report_path}'")

    # ==========================================
    # PLOTTING ENGINE (Using Averaged Data)
    # ==========================================
    print("Generating Averaged Plots...")

    # 1. Figure for each complexity level comparing planners (Averages)
    for idx, lvl in enumerate(complexity_lvls):
        # Only plot planners that had at least one success at this level
        names = [name for name in planners_classical if success_rates[name][idx] > 0]
        if not names: continue
        
        times = [avg_times[name][idx] for name in names]
        r_steps = [avg_r_steps[name][idx] for name in names]
        s_steps = [avg_s_steps[name][idx] for name in names]

        fig, ax1 = plt.subplots(figsize=(9, 6))
        bars = ax1.bar(names, times, color='skyblue', alpha=0.9, width=0.4, label='Avg Compute Time')
        ax1.set_xlabel('Planner', fontweight='bold')
        ax1.set_ylabel('Average Computation Time (s)', color='tab:blue', fontweight='bold')
        ax1.tick_params(axis='y', labelcolor='tab:blue')
        ax1.set_title(f'Average Performance ({lvl} Locations Map | {len(SEEDS)} Seeds)', fontweight='bold')

        for bar in bars:
            yval = bar.get_height()
            ax1.text(bar.get_x() + bar.get_width()/2, yval, f'{yval:.2f}s', ha='center', va='bottom', fontsize=9)

        ax2 = ax1.twinx()
        line1, = ax2.plot(names, r_steps, color='crimson', marker='o', linestyle='dashed', linewidth=2, markersize=8, label='Avg Replanning Steps')
        line2, = ax2.plot(names, s_steps, color='purple', marker='s', linestyle='-', linewidth=2, markersize=8, label='Avg Actions Executed')
        ax2.set_ylabel('Average Count (Steps / Actions)', color='black', fontweight='bold')
        
        lines = [bars, line1, line2]
        labels = [l.get_label() for l in lines]
        ax1.legend(lines, labels, loc='upper left')

        fig.tight_layout()
        plt.savefig(f'{figures_dir}/avg_complexity_{lvl}_comparison.png')
        plt.close()

    # 2. Figure for each planner showing evolution with complexity (Averages)
    for name in planners_classical:
        if sum(success_rates[name]) == 0: continue
        
        fig, ax1 = plt.subplots(figsize=(9, 6))
        times = avg_times[name]
        r_steps = avg_r_steps[name]
        s_steps = avg_s_steps[name]
        x_labels = [str(lvl) for lvl in complexity_lvls]

        bars = ax1.bar(x_labels, times, color='skyblue', alpha=0.9, width=0.5, label='Avg Compute Time')
        ax1.set_xlabel('Number of Locations (N)', fontweight='bold')
        ax1.set_ylabel('Average Computation Time (s)', color='tab:blue', fontweight='bold')
        ax1.tick_params(axis='y', labelcolor='tab:blue')
        ax1.set_title(f'{name} - Average Performance Evolution (Across {len(SEEDS)} Seeds)', fontweight='bold')

        for bar in bars:
            yval = bar.get_height()
            if yval > 0:
                ax1.text(bar.get_x() + bar.get_width()/2, yval, f'{yval:.1f}s', ha='center', va='bottom', fontsize=9)

        ax2 = ax1.twinx()
        line1, = ax2.plot(x_labels, r_steps, color='crimson', marker='o', linestyle='dashed', linewidth=2, markersize=8, label='Avg Replanning Steps')
        line2, = ax2.plot(x_labels, s_steps, color='purple', marker='s', linestyle='-', linewidth=2, markersize=8, label='Avg Actions Executed')
        ax2.set_ylabel('Average Count (Steps / Actions)', color='black', fontweight='bold')

        lines = [bars, line1, line2]
        labels = [l.get_label() for l in lines]
        ax1.legend(lines, labels, loc='upper left')

        fig.tight_layout()
        plt.savefig(f'{figures_dir}/{name}_avg_evolution.png')
        plt.close()

    print(f"All averaged plots have been saved successfully in the '{figures_dir}' folder!")

if __name__ == "__main__":
    benchmark()