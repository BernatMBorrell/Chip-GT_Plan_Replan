from unified_planning.shortcuts import *
from unified_planning.io import PDDLWriter

def generate_classic_pddl(num_nodes, folder, knowledge=None, unique_id="default", agents_state=None):
    # Create the problem with a UNIQUE name as requested
    problem_name = f"chip_gt_problem_{unique_id}"
    problem = Problem(problem_name)
    
    location = UserType("Location")
    trap = UserType("trap")
    robot = UserType("robot")
    ranger = UserType("ranger")
    animal = UserType("animal")

    # Define fluents
    problem.add_fluent("clear", BoolType(), l=location)
    problem.add_fluent("trap_at", BoolType(), t=trap, l=location)
    problem.add_fluent("robot_at", BoolType(), r=robot, l=location)
    problem.add_fluent("ranger_at", BoolType(), r=ranger, l=location)
    problem.add_fluent("adjacent", BoolType(), l1=location, l2=location)
    problem.add_fluent("traversable", BoolType(), l1=location, l2=location)
    problem.add_fluent("spot", BoolType(), r=robot)
    problem.add_fluent("drone", BoolType(), r=robot)
    problem.add_fluent("trap_push", BoolType(), t=trap)
    problem.add_fluent("trap_pic", BoolType(), t=trap)
    problem.add_fluent("trap_animal", BoolType(), t=trap)
    problem.add_fluent("animal_at", BoolType(), a=animal, l=location)
    problem.add_fluent("robot_carrying_animal", BoolType(), r=robot, a=animal)
    problem.add_fluent("ranger_carrying_animal", BoolType(), r=ranger, a=animal)

    # Define objects (fluents helpers)
    robot_at = problem.fluent("robot_at")
    adjacent = problem.fluent("adjacent")
    traversable = problem.fluent("traversable")
    spot = problem.fluent("spot")
    drone = problem.fluent("drone")
    ranger_at = problem.fluent("ranger_at")
    trap_at = problem.fluent("trap_at")
    trap_push = problem.fluent("trap_push")
    clear = problem.fluent("clear")
    trap_pic = problem.fluent("trap_pic")
    trap_animal = problem.fluent("trap_animal")
    animal_at = problem.fluent("animal_at")
    robot_carrying_animal = problem.fluent("robot_carrying_animal")
    ranger_carrying_animal = problem.fluent("ranger_carrying_animal")

    # --- ACTIONS DEFINITION ---

    # Action move ground
    move_ground = InstantaneousAction("move_ground", from_=location, to=location, r=robot)
    from_ = move_ground.parameter("from_")
    to = move_ground.parameter("to")
    r = move_ground.parameter("r")
    move_ground.add_precondition(And(robot_at(r, from_), Not(robot_at(r, to)), adjacent(from_, to), traversable(from_, to), spot(r)))
    move_ground.add_effect(robot_at(r, to), True)
    move_ground.add_effect(robot_at(r, from_), False)
    problem.add_action(move_ground)

    # Action move air
    move_air = InstantaneousAction("move_air", from_=location, to=location, r=robot)
    from_ = move_air.parameter("from_")
    to = move_air.parameter("to")
    r = move_air.parameter("r")
    move_air.add_precondition(And(robot_at(r, from_), Not(robot_at(r, to)), drone(r)))
    move_air.add_effect(robot_at(r, to), True)
    move_air.add_effect(robot_at(r, from_), False)
    problem.add_action(move_air)

    # Action move ranger
    move_ranger = InstantaneousAction("move_ranger", from_=location, to=location, r=ranger)
    from_ = move_ranger.parameter("from_")
    to = move_ranger.parameter("to")
    r = move_ranger.parameter("r")
    move_ranger.add_precondition(And(ranger_at(r, from_), Not(ranger_at(r, to)), adjacent(from_, to)))
    move_ranger.add_effect(ranger_at(r, to), True)
    move_ranger.add_effect(ranger_at(r, from_), False)
    problem.add_action(move_ranger)

    # Action robot_disarm_trap_push
    act_r_push = InstantaneousAction("robot_disarm_trap_push", t=trap, l=location, r=robot)
    t = act_r_push.parameter("t")
    l = act_r_push.parameter("l")
    r = act_r_push.parameter("r")
    act_r_push.add_precondition(And(robot_at(r, l), trap_at(t, l), trap_push(t), spot(r)))
    act_r_push.add_effect(clear(l), True)
    act_r_push.add_effect(trap_at(t, l), False)
    problem.add_action(act_r_push)

    # Action ranger_disarm_trap_push
    act_ran_push = InstantaneousAction("ranger_disarm_trap_push", t=trap, l=location, r=ranger)
    t = act_ran_push.parameter("t")
    l = act_ran_push.parameter("l")
    r = act_ran_push.parameter("r")
    act_ran_push.add_precondition(And(ranger_at(r, l), trap_at(t, l), trap_push(t)))
    act_ran_push.add_effect(clear(l), True)
    act_ran_push.add_effect(trap_at(t, l), False)
    problem.add_action(act_ran_push)

    # Action ranger_disarm_trap_pic
    act_ran_pic = InstantaneousAction("ranger_disarm_trap_pic", t=trap, l=location, r=robot)
    t = act_ran_pic.parameter("t")
    l = act_ran_pic.parameter("l")
    r = act_ran_pic.parameter("r")
    act_ran_pic.add_precondition(And(robot_at(r, l), trap_at(t, l), trap_pic(t)))
    act_ran_pic.add_effect(clear(l), True)
    act_ran_pic.add_effect(trap_at(t, l), False)
    problem.add_action(act_ran_pic)

    # Action disarm_trap_animal
    act_disarm_ani = InstantaneousAction("disarm_trap_animal", t=trap, l=location, r=robot)
    t = act_disarm_ani.parameter("t")
    l = act_disarm_ani.parameter("l")
    r = act_disarm_ani.parameter("r")
    act_disarm_ani.add_precondition(And(robot_at(r, l), trap_at(t, l), trap_animal(t)))
    act_disarm_ani.add_effect(clear(l), True)
    act_disarm_ani.add_effect(trap_at(t, l), False)
    problem.add_action(act_disarm_ani)

    # Action robot_carry_animal
    act_r_carry = InstantaneousAction("robot_carry_animal", l=location, r=robot, a=animal)
    l = act_r_carry.parameter("l")
    r = act_r_carry.parameter("r")
    a = act_r_carry.parameter("a")
    act_r_carry.add_precondition(And(robot_at(r, l), animal_at(a, l), clear(l), spot(r)))
    act_r_carry.add_effect(animal_at(a, l), False)
    act_r_carry.add_effect(robot_carrying_animal(r, a), True)
    problem.add_action(act_r_carry)

    # Action robot_deliver_animal
    act_r_deliver = InstantaneousAction("robot_deliver_animal", l=location, r=robot, a=animal)
    l = act_r_deliver.parameter("l")
    r = act_r_deliver.parameter("r")
    a = act_r_deliver.parameter("a")
    act_r_deliver.add_precondition(And(robot_at(r, l), robot_carrying_animal(r, a), spot(r)))
    act_r_deliver.add_effect(robot_carrying_animal(r, a), False)
    act_r_deliver.add_effect(animal_at(a, l), True)
    problem.add_action(act_r_deliver)

    # Action ranger_carry_animal
    act_ran_carry = InstantaneousAction("ranger_carry_animal", l=location, r=ranger, a=animal)
    l = act_ran_carry.parameter("l")
    r = act_ran_carry.parameter("r")
    a = act_ran_carry.parameter("a")
    act_ran_carry.add_precondition(And(ranger_at(r, l), animal_at(a, l), clear(l)))
    act_ran_carry.add_effect(animal_at(a, l), False)
    act_ran_carry.add_effect(ranger_carrying_animal(r, a), True)
    problem.add_action(act_ran_carry)

    # Action ranger_deliver_animal
    act_ran_deliver = InstantaneousAction("ranger_deliver_animal", l=location, r=ranger, a=animal)
    l = act_ran_deliver.parameter("l")
    r = act_ran_deliver.parameter("r")
    a = act_ran_deliver.parameter("a")
    act_ran_deliver.add_precondition(And(ranger_at(r, l), ranger_carrying_animal(r, a)))
    act_ran_deliver.add_effect(ranger_carrying_animal(r, a), False)
    act_ran_deliver.add_effect(animal_at(a, l), True)
    problem.add_action(act_ran_deliver)

    # --- OBJECTS INSTANTIATION ---
    locations = {loc_name: problem.add_object(loc_name, location) for loc_name in [f"l_{i:02d}" for i in range(1, num_nodes+1)] + ["l_start", "l_end"]}
    robots = {rob_name: problem.add_object(rob_name, robot) for rob_name in ["r_drone", "r_spot"]}
    traps = {trap_name: problem.add_object(trap_name, trap) for trap_name in [f"t_{i:02d}" for i in range(1, num_nodes+1)]}
    animals = {animal_name: problem.add_object(animal_name, animal) for animal_name in ["a_01"]}
    rangers = {ranger_name: problem.add_object(ranger_name, ranger) for ranger_name in ["caro", "bapt"]}

    # --- INIT: STATIC FACTS ---
    problem.set_initial_value(drone(robots["r_drone"]), True)
    problem.set_initial_value(spot(robots["r_spot"]), True)
    
    default_state = {
        "r_drone": "l_start",
        "r_spot": "l_start",
        "caro": "l_start",
        "bapt": "l_start"
    }

    current_state = default_state.copy()
    if agents_state:
        current_state.update(agents_state)

    for agent_name, loc_name in current_state.items():
        loc_obj = locations[loc_name]
        
        if agent_name in robots:
            rob_obj = robots[agent_name]
            problem.set_initial_value(robot_at(rob_obj, loc_obj), True)
        elif agent_name in rangers:
            ran_obj = rangers[agent_name]
            problem.set_initial_value(ranger_at(ran_obj, loc_obj), True)

    # INIT: Adjacency (Linear Graph)
    for i in range(1, num_nodes+1):
        loc = f"l_{i:02d}"
        if i == 1:
            problem.set_initial_value(adjacent(locations["l_start"], locations[loc]), True)
            problem.set_initial_value(adjacent(locations[loc], locations["l_start"]), True)
            problem.set_initial_value(traversable(locations["l_start"], locations[loc]), True)
            problem.set_initial_value(traversable(locations[loc], locations["l_start"]), True)
        if i == num_nodes:
            problem.set_initial_value(adjacent(locations[loc], locations["l_end"]), True)
            problem.set_initial_value(adjacent(locations["l_end"], locations[loc]), True)
            problem.set_initial_value(traversable(locations[loc], locations["l_end"]), True)
            problem.set_initial_value(traversable(locations["l_end"], locations[loc]), True)
        if i > 1:
            prev = f"l_{i-1:02d}"
            problem.set_initial_value(adjacent(locations[loc], locations[prev]), True)
            problem.set_initial_value(adjacent(locations[prev], locations[loc]), True)
            problem.set_initial_value(traversable(locations[loc], locations[prev]), True)
            problem.set_initial_value(traversable(locations[prev], locations[loc]), True)

    # --- INIT: TRAPS AND KNOWLEDGE LOGIC ---
    for i in range(1, num_nodes+1):
        trap_name = f"t_{i:02d}"
        loc_name = f"l_{i:02d}"
        trap = traps[trap_name]
        loc = locations[loc_name]

        # 1. GROUND TRUTH (Logic used for Simulation)
        real_trap_type = "trap_pic" 
        if i % 3 == 1: real_trap_type = "trap_push"
        elif i % 3 == 2: real_trap_type = "trap_animal"
        
        # 2. DECISION (Sim vs Planner)
        should_add_trap = False
        final_trap_type = None

        if knowledge is None:
            # Simulation Mode: Add everything
            should_add_trap = True
            final_trap_type = real_trap_type
        else:
            # Planner Mode: Use knowledge dict
            state = knowledge.get(loc_name, "unknown")
            if state == "unknown" or state == "clear":
                # Optimistic: Assume clear
                problem.set_initial_value(clear(loc), True)
                should_add_trap = False
            elif state.startswith("trap_"):
                # Knowledge: Add trap
                should_add_trap = True
                final_trap_type = state
        
        # 3. APPLY
        if should_add_trap:
            problem.set_initial_value(trap_at(trap, loc), True)
            if final_trap_type == "trap_push":
                problem.set_initial_value(trap_push(trap), True)
            elif final_trap_type == "trap_animal":
                problem.set_initial_value(trap_animal(trap), True)
            elif final_trap_type == "trap_pic":
                problem.set_initial_value(trap_pic(trap), True)

    # INIT: Animal (Fixed at start/l_02 for simplicity)
    animal_loc = "l_02" if num_nodes >= 2 else "l_01"
    problem.set_initial_value(animal_at(animals["a_01"], locations[animal_loc]), True)

    # GOAL
    clear_goals = [clear(locations[f"l_{i:02d}"]) for i in range(1, num_nodes+1)]
    problem.add_goal(And(
        *clear_goals,
        robot_at(robots["r_drone"], locations["l_end"]),
        robot_at(robots["r_spot"], locations["l_end"]),
        ranger_at(rangers["caro"], locations["l_end"]),
        ranger_at(rangers["bapt"], locations["l_end"]),
        animal_at(animals["a_01"], locations["l_start"])
    ))

    # --- MODIFIED: NO DOMAIN WRITING, NO PRINT SPAM ---
    # We return the object directly. 
    # If you really need the file, we write only the problem file silently.
    
    # writer = PDDLWriter(problem)
    # writer.write_problem(f"{folder}/problem_{unique_id}.pddl") # Optional: write unique problem
    
    return problem