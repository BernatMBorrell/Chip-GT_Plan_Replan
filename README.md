# Chip-GT_Plan_Replan
A Python-based simulation demonstrating Concurrent Planning and Execution under Partial Observability using classical AI planners. 

This project uses the Unified Planning (UP) framework and the Fast Downward engine to coordinate a heterogeneous team of agents (a drone, a ground rover, and human rangers). The system dynamically handles "Fog of War" (unknown terrain) by using the Free-Space Assumption, forcing agents to scout ahead, and triggering real-time replanning when unexpected obstacles (traps) are discovered. The mission is only considered complete when 100% of the map has been explored, all traps are cleared, the animal is rescued, and all agents reach the extraction point.

## Key Features

* **Dynamic Replanning**: The planner assumes unknown nodes are safe (optimistic assumption) but enforces a strict rule: ground agents cannot enter a node until it is formally inspected. If a scout discovers a trap, execution halts, the knowledge base updates, and a new plan is computed on the fly.
* **Total Exploration**: The PDDL goals explicitly require every node in the graph to lose its 'unknown' status, forcing the AI to methodically map the entire environment.
* **Heterogeneous Agents & Specialization**: Agents have distinct physical constraints and capabilities, forcing the AI to orchestrate cooperative strategies.
* **Graph-Based Topology**: The world is built using NetworkX, featuring complex, interconnected nodes. Some paths are adjacent (traversable by humans) but not traversable (blocked for the heavy rover).
* **In-Memory PDDL Generation**: Bypasses slow I/O operations by dynamically generating UP Problem objects directly in RAM.
* **Built-in Visualization Engine**: Automatically records the mission and exports a step-by-step animated GIF of the graph, showing agent movements, fog of war, and discovered traps.

## The Squad (Agents)

| Agent | Movement | Capabilities | Limitations |
| :--- | :--- | :--- | :--- |
| **Drone (r_drone)** | `move_air`: Direct flight. Ignores graph topology. | `inspect`: Fastest scout. | Cannot disarm traps or carry animals. |
| **Spot (r_spot)** | `move_ground`: Requires traversable and inspected paths. | `inspect`, `disarm_trap_push`, `carry_animal`. | Blocked by non-traversable terrain. Cannot disarm light traps. |
| **Caro & Bapt (Rangers)** | `move_ranger`: Requires adjacent and inspected paths. | `inspect`, `disarm_trap_pic`, `disarm_trap_animal`, `carry_animal`. | Slower movement (must follow graph topology node-by-node). |

## Project Structure

* `graphgenerator.py`: Generates the physical graph network (nodes, edges, and traversability limits) using NetworkX.
* `domaingenerator.py`: Contains the PDDL domain definition built via the Unified Planning API. It translates current world knowledge into fluents, objects, and specialized agent actions.
* `timedsim.py`: The main execution loop. It generates the ground truth, asks the planner for a strategy, executes actions, handles sensor interruptions (scouting), and triggers the GIF rendering at the end.

## How It Works (The Execution Loop)

1. **Initial State**: All nodes (except Start and End) are marked as unknown. 
2. **Planning Phase**: The AI computes a plan to reach the goals. Because ground agents are "blind", it delegates the inspect actions to the Drone, which flies ahead.
3. **Execution & Discovery**: Python executes the planned actions. When the inspect action occurs, the simulator checks the "Ground Truth".
4. **Interruption**: If the inspection reveals a trap, the optimistic plan is invalidated. Execution stops.
5. **Replanning**: The loop restarts. The AI now knows about the trap and will dispatch the correct specialized agent to clear it before continuing.

## Installation

1. Create and activate a Python virtual environment (recommended):
   ```bash
   python3 -m venv rp_env
   source rp_env/bin/activate

2. Install Dependencies:
   ```bash
   pip install unified-planning networkx matplotlib
   ```

3. Install the planning engine (Fast Downward is recommended for this domain)::
   ```bash
   pip install up-fast-downward
   ```

4. Run the Simulation:
   ```bash
   python3 timedsim.py
   ```

## How to Modify the Mission

You can easily change the map layout, trap locations, and agent capabilities by editing the `graphgenerator.py` and `domaingenerator.py` files.

### Key Configuration Points

* **Map Size**: Adjust `num_nodes` in `graphgenerator.py`.
* **Traversability**: Modify the `traversable` dictionary in `graphgenerator.py` to block or allow paths for ground agents.
* **Trap Distribution**: Edit the `knowledge` dictionary in `timedsim.py` to pre-place traps (or remove them) for testing.
* **Agent Capabilities**: In `domaingenerator.py`, you can adjust the `cost` or `preconditions` of actions to make agents stronger or weaker.

## License

This project is for educational purposes, developed as part of the ISAE-SUPAERO "Robotics and Planning" course.
