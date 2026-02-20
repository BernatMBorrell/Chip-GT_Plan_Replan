from unified_planning.shortcuts import *

def generate_classic_pddl(graph, knowledge, unique_id="default", agents_state=None, animal_state="l_02"):
    problem = Problem(f"chip_gt_problem_{unique_id}")
    
    # TYPES
    location = UserType("Location")
    trap = UserType("trap")
    robot = UserType("robot")
    ranger = UserType("ranger")
    animal = UserType("animal")

    # FLUENTS
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
    
    problem.add_fluent("inspected", BoolType(), l=location) 
    problem.add_fluent("unknown_loc", BoolType(), l=location) 

    # Fluent helpers
    robot_at = problem.fluent("robot_at")
    ranger_at = problem.fluent("ranger_at")
    adjacent = problem.fluent("adjacent")
    traversable = problem.fluent("traversable")
    spot = problem.fluent("spot")
    drone = problem.fluent("drone")
    trap_at = problem.fluent("trap_at")
    trap_push = problem.fluent("trap_push")
    trap_pic = problem.fluent("trap_pic")
    trap_animal = problem.fluent("trap_animal")
    clear = problem.fluent("clear")
    animal_at = problem.fluent("animal_at")
    robot_carrying_animal = problem.fluent("robot_carrying_animal")
    ranger_carrying_animal = problem.fluent("ranger_carrying_animal")
    inspected = problem.fluent("inspected")
    unknown_loc = problem.fluent("unknown_loc")

    # --- ACTIONS ---

    # 1. DRONE INSPECT: The drone inspects the node IT IS CURRENTLY AT.
    drone_inspect = InstantaneousAction("drone_inspect", pos=location, r=robot)
    drone_inspect.add_precondition(And(robot_at(drone_inspect.parameter("r"), drone_inspect.parameter("pos")), 
                                       drone(drone_inspect.parameter("r")), 
                                       unknown_loc(drone_inspect.parameter("pos"))))
    drone_inspect.add_effect(unknown_loc(drone_inspect.parameter("pos")), False)
    drone_inspect.add_effect(inspected(drone_inspect.parameter("pos")), True)
    problem.add_action(drone_inspect)

    # 2. SPOT INSPECT: Spot inspects an ADJACENT node (minesweeper style)
    spot_inspect = InstantaneousAction("spot_inspect", pos=location, target=location, r=robot)
    spot_inspect.add_precondition(And(robot_at(spot_inspect.parameter("r"), spot_inspect.parameter("pos")), 
                                      spot(spot_inspect.parameter("r")),
                                      adjacent(spot_inspect.parameter("pos"), spot_inspect.parameter("target")), 
                                      unknown_loc(spot_inspect.parameter("target"))))
    spot_inspect.add_effect(unknown_loc(spot_inspect.parameter("target")), False)
    spot_inspect.add_effect(inspected(spot_inspect.parameter("target")), True)
    problem.add_action(spot_inspect)

    # 3. RANGER INSPECT: Rangers inspect an ADJACENT node
    ranger_inspect = InstantaneousAction("ranger_inspect", pos=location, target=location, ran=ranger)
    ranger_inspect.add_precondition(And(ranger_at(ranger_inspect.parameter("ran"), ranger_inspect.parameter("pos")), 
                                        adjacent(ranger_inspect.parameter("pos"), ranger_inspect.parameter("target")), 
                                        unknown_loc(ranger_inspect.parameter("target"))))
    ranger_inspect.add_effect(unknown_loc(ranger_inspect.parameter("target")), False)
    ranger_inspect.add_effect(inspected(ranger_inspect.parameter("target")), True)
    problem.add_action(ranger_inspect)

    # MOVEMENTS
    move_ground = InstantaneousAction("move_ground", from_=location, to=location, r=robot)
    move_ground.add_precondition(And(robot_at(move_ground.parameter("r"), move_ground.parameter("from_")), 
                                     Not(robot_at(move_ground.parameter("r"), move_ground.parameter("to"))), 
                                     adjacent(move_ground.parameter("from_"), move_ground.parameter("to")), 
                                     traversable(move_ground.parameter("from_"), move_ground.parameter("to")), 
                                     spot(move_ground.parameter("r")), 
                                     inspected(move_ground.parameter("to")))) 
    move_ground.add_effect(robot_at(move_ground.parameter("r"), move_ground.parameter("to")), True)
    move_ground.add_effect(robot_at(move_ground.parameter("r"), move_ground.parameter("from_")), False)
    problem.add_action(move_ground)

    move_air = InstantaneousAction("move_air", from_=location, to=location, r=robot)
    move_air.add_precondition(And(robot_at(move_air.parameter("r"), move_air.parameter("from_")), 
                                  Not(robot_at(move_air.parameter("r"), move_air.parameter("to"))), 
                                  drone(move_air.parameter("r"))))
    move_air.add_effect(robot_at(move_air.parameter("r"), move_air.parameter("to")), True)
    move_air.add_effect(robot_at(move_air.parameter("r"), move_air.parameter("from_")), False)
    problem.add_action(move_air)

    move_ranger = InstantaneousAction("move_ranger", from_=location, to=location, ran=ranger)
    move_ranger.add_precondition(And(ranger_at(move_ranger.parameter("ran"), move_ranger.parameter("from_")), 
                                     Not(ranger_at(move_ranger.parameter("ran"), move_ranger.parameter("to"))), 
                                     adjacent(move_ranger.parameter("from_"), move_ranger.parameter("to")),
                                     inspected(move_ranger.parameter("to")))) 
    move_ranger.add_effect(ranger_at(move_ranger.parameter("ran"), move_ranger.parameter("to")), True)
    move_ranger.add_effect(ranger_at(move_ranger.parameter("ran"), move_ranger.parameter("from_")), False)
    problem.add_action(move_ranger)

    # DISARMS
    act_r_push = InstantaneousAction("robot_disarm_trap_push", l=location, r=robot, t=trap)
    act_r_push.add_precondition(And(robot_at(act_r_push.parameter("r"), act_r_push.parameter("l")), trap_at(act_r_push.parameter("t"), act_r_push.parameter("l")), trap_push(act_r_push.parameter("t")), spot(act_r_push.parameter("r"))))
    act_r_push.add_effect(clear(act_r_push.parameter("l")), True)
    act_r_push.add_effect(trap_at(act_r_push.parameter("t"), act_r_push.parameter("l")), False)
    problem.add_action(act_r_push)

    act_ran_pic = InstantaneousAction("ranger_disarm_trap_pic", l=location, ran=ranger, t=trap)
    act_ran_pic.add_precondition(And(ranger_at(act_ran_pic.parameter("ran"), act_ran_pic.parameter("l")), trap_at(act_ran_pic.parameter("t"), act_ran_pic.parameter("l")), trap_pic(act_ran_pic.parameter("t"))))
    act_ran_pic.add_effect(clear(act_ran_pic.parameter("l")), True)
    act_ran_pic.add_effect(trap_at(act_ran_pic.parameter("t"), act_ran_pic.parameter("l")), False)
    problem.add_action(act_ran_pic)

    act_ran_ani = InstantaneousAction("ranger_disarm_trap_animal", l=location, ran=ranger, t=trap)
    act_ran_ani.add_precondition(And(ranger_at(act_ran_ani.parameter("ran"), act_ran_ani.parameter("l")), trap_at(act_ran_ani.parameter("t"), act_ran_ani.parameter("l")), trap_animal(act_ran_ani.parameter("t"))))
    act_ran_ani.add_effect(clear(act_ran_ani.parameter("l")), True)
    act_ran_ani.add_effect(trap_at(act_ran_ani.parameter("t"), act_ran_ani.parameter("l")), False)
    problem.add_action(act_ran_ani)

    # ANIMAL HANDLING
    act_r_carry = InstantaneousAction("robot_carry_animal", l=location, r=robot, a=animal)
    act_r_carry.add_precondition(And(robot_at(act_r_carry.parameter("r"), act_r_carry.parameter("l")), animal_at(act_r_carry.parameter("a"), act_r_carry.parameter("l")), clear(act_r_carry.parameter("l")), spot(act_r_carry.parameter("r"))))
    act_r_carry.add_effect(animal_at(act_r_carry.parameter("a"), act_r_carry.parameter("l")), False)
    act_r_carry.add_effect(robot_carrying_animal(act_r_carry.parameter("r"), act_r_carry.parameter("a")), True)
    problem.add_action(act_r_carry)

    act_r_deliver = InstantaneousAction("robot_deliver_animal", l=location, r=robot, a=animal)
    act_r_deliver.add_precondition(And(robot_at(act_r_deliver.parameter("r"), act_r_deliver.parameter("l")), robot_carrying_animal(act_r_deliver.parameter("r"), act_r_deliver.parameter("a")), spot(act_r_deliver.parameter("r"))))
    act_r_deliver.add_effect(robot_carrying_animal(act_r_deliver.parameter("r"), act_r_deliver.parameter("a")), False)
    act_r_deliver.add_effect(animal_at(act_r_deliver.parameter("a"), act_r_deliver.parameter("l")), True)
    problem.add_action(act_r_deliver)

    act_ran_carry = InstantaneousAction("ranger_carry_animal", l=location, ran=ranger, a=animal)
    act_ran_carry.add_precondition(And(ranger_at(act_ran_carry.parameter("ran"), act_ran_carry.parameter("l")), animal_at(act_ran_carry.parameter("a"), act_ran_carry.parameter("l")), clear(act_ran_carry.parameter("l"))))
    act_ran_carry.add_effect(animal_at(act_ran_carry.parameter("a"), act_ran_carry.parameter("l")), False)
    act_ran_carry.add_effect(ranger_carrying_animal(act_ran_carry.parameter("ran"), act_ran_carry.parameter("a")), True)
    problem.add_action(act_ran_carry)

    act_ran_deliver = InstantaneousAction("ranger_deliver_animal", l=location, ran=ranger, a=animal)
    act_ran_deliver.add_precondition(And(ranger_at(act_ran_deliver.parameter("ran"), act_ran_deliver.parameter("l")), ranger_carrying_animal(act_ran_deliver.parameter("ran"), act_ran_deliver.parameter("a"))))
    act_ran_deliver.add_effect(ranger_carrying_animal(act_ran_deliver.parameter("ran"), act_ran_deliver.parameter("a")), False)
    act_ran_deliver.add_effect(animal_at(act_ran_deliver.parameter("a"), act_ran_deliver.parameter("l")), True)
    problem.add_action(act_ran_deliver)

    # OBJECTS & INIT
    loc_objs = {n: problem.add_object(n, location) for n in graph.graph.nodes()}
    robots = {"r_drone": problem.add_object("r_drone", robot), "r_spot": problem.add_object("r_spot", robot)}
    rangers = {"caro": problem.add_object("caro", ranger), "bapt": problem.add_object("bapt", ranger)}
    traps = {f"t_{i:02d}": problem.add_object(f"t_{i:02d}", trap) for i in range(1, len(graph.locations)+1)}
    animal_obj = problem.add_object("a_01", animal)

    problem.set_initial_value(drone(robots["r_drone"]), True)
    problem.set_initial_value(spot(robots["r_spot"]), True)
    
    current_state = {"r_drone": "l_start", "r_spot": "l_start", "caro": "l_start", "bapt": "l_start"}
    if agents_state: current_state.update(agents_state)

    for agent, loc in current_state.items():
        if agent in robots: problem.set_initial_value(robot_at(robots[agent], loc_objs[loc]), True)
        else: problem.set_initial_value(ranger_at(rangers[agent], loc_objs[loc]), True)

    for u, v in graph.graph.edges():
        problem.set_initial_value(adjacent(loc_objs[u], loc_objs[v]), True)
        problem.set_initial_value(adjacent(loc_objs[v], loc_objs[u]), True)
    for u, v in graph.traversable_edges:
        problem.set_initial_value(traversable(loc_objs[u], loc_objs[v]), True)
        problem.set_initial_value(traversable(loc_objs[v], loc_objs[u]), True)

    problem.set_initial_value(inspected(loc_objs["l_start"]), True)
    problem.set_initial_value(unknown_loc(loc_objs["l_start"]), False)
    problem.set_initial_value(clear(loc_objs["l_start"]), True)
    
    problem.set_initial_value(inspected(loc_objs["l_end"]), True)
    problem.set_initial_value(unknown_loc(loc_objs["l_end"]), False)
    problem.set_initial_value(clear(loc_objs["l_end"]), True)
    
    for loc_name in [l.name for l in graph.locations]:
        loc_obj = loc_objs[loc_name]
        trap_name = f"t_{loc_name.split('_')[1]}"
        trap_obj = traps[trap_name]
        state = knowledge.get(loc_name, "unknown")
        
        if state == "unknown":
            problem.set_initial_value(unknown_loc(loc_obj), True)
            problem.set_initial_value(inspected(loc_obj), False)
            problem.set_initial_value(clear(loc_obj), True)
        else:
            problem.set_initial_value(unknown_loc(loc_obj), False)
            problem.set_initial_value(inspected(loc_obj), True)
            if state == "clear":
                problem.set_initial_value(clear(loc_obj), True)
            elif state.startswith("trap_"):
                problem.set_initial_value(trap_at(trap_obj, loc_obj), True)
                if "push" in state: problem.set_initial_value(trap_push(trap_obj), True)
                if "pic" in state: problem.set_initial_value(trap_pic(trap_obj), True)
                if "animal" in state: problem.set_initial_value(trap_animal(trap_obj), True)

    if animal_state.startswith("carried_by_"):
        carrier = animal_state.replace("carried_by_", "")
        if carrier in robots:
            problem.set_initial_value(robot_carrying_animal(robots[carrier], animal_obj), True)
        else:
            problem.set_initial_value(ranger_carrying_animal(rangers[carrier], animal_obj), True)
    else:
        problem.set_initial_value(animal_at(animal_obj, loc_objs[animal_state]), True)
    # 7. GOALS
    # Goal 1: All traps must be disarmed (clear)
    clear_goals = [clear(loc_objs[l.name]) for l in graph.locations]
    
    # Goal 2: ALL FOG OF WAR MUST BE CLEARED (Every node must be inspected)
    inspected_goals = [inspected(loc_objs[l.name]) for l in graph.locations]
    
    problem.add_goal(And(
        *clear_goals,
        *inspected_goals,  # <--- THIS FORCES TOTAL MAP EXPLORATION
        robot_at(robots["r_drone"], loc_objs["l_end"]),
        robot_at(robots["r_spot"], loc_objs["l_end"]),
        ranger_at(rangers["caro"], loc_objs["l_end"]),
        ranger_at(rangers["bapt"], loc_objs["l_end"]),
        animal_at(animal_obj, loc_objs["l_start"])
    ))

    return problem