#!/usr/bin/env python

import rclpy
import subprocess
import json
import threading
from string import Template
from rclpy.executors import MultiThreadedExecutor
from rclpy.signals import SignalHandlerOptions
from ros2ample.ample_up import AmpleUPPlanningNode

# execution imports
from ugv_ranger_skillset_client import UgvRangerSkillsetClient
from uav_ranger_skillset_client import UavRangerSkillsetClient
from team_controller_client.composites import *
from team_controller_client.team_controller import TeamController
# from team_controller_alpha_client.team_controller_alpha import TeamControllerAlpha
# from team_controller_beta_client.team_controller_beta import TeamControllerBeta
from webots_ros2_manager_msgs.msg import Objective


class AmpleUPImplNode(AmpleUPPlanningNode):
    """
    AMPLIFIED plan-replan implementation.
    """

    def __init__(self, node_name: str = "ample"):
        super().__init__(node_name=node_name)

        self._controller_is_ready: threading.Event = threading.Event()
        self._init_timer = self.create_timer(1000,self._init_controller)

        # replanning data
        self.trap_type_mapping = {
            Objective.IS_COMBINED: "t-combi",
            Objective.IS_AIR: "t-air",
            Objective.IS_GROUND: "t-ground",
        }
        self.found_trap_types = {}
        self.unreachable_locs = []
        self.mission_status = {}

    def _init_controller(self):
        # init clients
        if self.database["robots"] == {}:
            self.get_logger().info("Waiting for database to send robots...")
            return

        self.get_logger().info("Connecting to robots clients")
        # if self.assigned_team == "alpha":
        #     self.composites_controller = TeamControllerAlpha(no_clients=True)
        # elif self.assigned_team == "beta":
        #     self.composites_controller = TeamControllerBeta(no_clients=True)
        # else:
        self.composites_controller = TeamController(no_clients=True)

        i_robot = 0
        n_robots = len(self.database["robots"].keys())
        for robot_name, robot_dict in self.database["robots"].items():
            i_robot += 1
            self.get_logger().info(f"[{str(i_robot)}/{str(n_robots)}] {robot_name}")
            if robot_dict["kind"] == "ugv":
                self.robot_clients[robot_name] = UgvRangerSkillsetClient(
                    node_name="ugv_skillset_client",
                    skillset_manager=robot_dict["manager"],
                    data_subscription=True)
            elif robot_dict["kind"] == "uav":
                self.robot_clients[robot_name] = UavRangerSkillsetClient(
                    node_name="uav_skillset_client",
                    skillset_manager=robot_dict["manager"],
                    data_subscription=True)
        self.get_logger().info("Clients ready!")
        self._controller_is_ready.set()
        self._init_timer.cancel()

    def update_problem_template(self):
        with open(self.problem, "r") as f:
            template = Template(f.read())
        d = {
            "locations": "",
            "robots": "",
            "objectives": "",
            "robots_types": "",
            "robots_locations": "",
            "objectives_locations": "",
            "unreachable_locations": "",
            "objectives_detect_types": "",
            "goals": "",
        }

        for loc in self.database["locations"].keys():
            if loc.startswith(self.database["data"]["area"]["id"]):
                d["locations"] += f"{loc} "
                if loc in self.unreachable_locs:
                    d["unreachable_locations"] += f"(unreachable {loc})"
        d["locations"] += "- location"

        for obj, obj_dict in self.database["objectives"].items():
            if obj.startswith(self.database["data"]["area"]["id"]):
                if obj_dict["data"]["status"] == Objective.ACTIVE:
                    if obj in self.found_trap_types.keys():
                        if self.found_trap_types[obj] in self.trap_type_mapping.keys():
                            d["objectives"] += f"{obj} "
                            # trap exists and has a type
                            trap_type_pred = self.trap_type_mapping[
                                self.found_trap_types[obj]
                            ]
                            d["objectives_detect_types"] += f"({trap_type_pred} {obj}) (detected {obj}) "
                            d["objectives_locations"] += f"(t-at {obj} {obj_dict["location"]}) "
                            # check soft goal
                            if obj_dict["location"] not in self.unreachable_locs:
                                d["goals"] += f"(safe {obj_dict["location"]}) "
                        else:
                            # type was NO_TRAP: does not exist
                            pass
                    else:
                        # by default, consider traps as combined type
                        d["objectives"] += f"{obj} "
                        d["objectives_detect_types"] += f"(t-combi {obj}) "
                        d["objectives_locations"] += f"(t-at {obj} {obj_dict["location"]}) "
                        d["goals"] += f"(safe {obj_dict["location"]}) "

        if d["objectives"] != "":
            d["objectives"] += "- trap"

        for robot_name, robot_dict in self.database["robots"].items():
            d["robots"] += f"{robot_name} "
            d["robots_types"] += f"(r-{robot_dict["kind"]} {robot_name}) "
            d["robots_locations"] += f"(r-at {robot_name} {robot_dict["location"]}) "
        d["robots"] += "- robot"

        self.templated_problem = "/tmp/"+self.assigned_team+"temp_problem.pddl"
        self.last_saved_problem = template.safe_substitute(d)
        with open(self.templated_problem, "w") as f:
            f.write(self.last_saved_problem)

    def get_task_data(self, step: dict):
        task = step["Task"]
        action_name = task[:task.index("[")]
        preds = task[task.index("[")+1:task.index("]")].split(",")

        robots = []
        inputs = []
        for x in preds:
            if x in self.database["robots"].keys():
                robots.append(x)
            else:
                inputs.append(x)

        return action_name, robots, inputs
    
    def update_mission_status(self, plan_step:int,status:str,result=None):
        if result is None:
            self.mission_status[f"plan/action_{str(plan_step)}"] = {"running":status,"result":None}
        else:
            self.mission_status[f"plan/action_{str(plan_step)}"] = {"running":status,"result":{"_result_":result}}
        self.composites_controller.update_status(cs="plan",status=self.mission_status)


    def execute(self, id: str):
        if not self._controller_is_ready.is_set():
            self._controller_is_ready.wait()

        plan_dict = self.results[id][1]
        self.mission_status = {}

        for i,step in enumerate(plan_dict.values()):
            action_name, robots, inputs = self.get_task_data(step)
            self.get_logger().info(
                f"Plan step: {action_name}[{",".join(robots)}]({",".join(inputs)})"
            )

            if action_name == "move_uav":
                action = MoveUav(
                    controller=self.composites_controller,
                    uav_ranger_client=self.robot_clients[robots[0]]
                )
                target_loc = inputs[1]
                action_input = MoveUavInput()
                action_input.waypoint.x = self.database["locations"][target_loc]["data"]["x"]
                action_input.waypoint.y = self.database["locations"][target_loc]["data"]["y"]
                action.start(action_input)
                self.update_mission_status(i,"true")
                result = action.wait_result()
                self.update_mission_status(i,"false",action.result_to_str(result))
            

            elif action_name == "move_ugv":
                action = MoveUgv(
                    controller=self.composites_controller,
                    ugv_ranger_client=self.robot_clients[robots[0]]
                )
                target_loc = inputs[1]
                action_input = MoveUgvInput()
                action_input.waypoint.x = self.database["locations"][target_loc]["data"]["x"]
                action_input.waypoint.y = self.database["locations"][target_loc]["data"]["y"]
                action.start(action_input)
                self.update_mission_status(i,"true")
                result = action.wait_result()
                self.update_mission_status(i,"false",action.result_to_str(result))

                if result != action.SUCCESS__SUCCEEDED:
                    self.get_logger().info(
                        f"Location {target_loc} is unreachable.")
                    self.unreachable_locs.append(target_loc)
                    return False

            elif action_name == "detect_trap":
                action = DetectTrap(
                    controller=self.composites_controller,
                    uav_ranger_client=self.robot_clients[robots[0]]
                )
                target_trap = inputs[1]
                action_input = DetectTrapInput()
                action_input.trap.name = target_trap
                action_input.trap.position.x = self.database["objectives"][target_trap]["data"]["position"]["x"]
                action_input.trap.position.y = self.database["objectives"][target_trap]["data"]["position"]["y"]
                action.start(action_input)
                self.update_mission_status(i,"true")
                result = action.wait_result()
                self.update_mission_status(i,"false",action.result_to_str(result))

                trap_type = action.get_result(
                    "detect").output.objective_data.type
                self.found_trap_types[target_trap] = trap_type
                if trap_type != Objective.IS_COMBINED:
                    self.get_logger().info(
                        f"Trap {target_trap} is new type {trap_type}.")
                    return False

            elif action_name == "disarm_trap_ground":
                action = DisarmTrapGround(
                    controller=self.composites_controller,
                    ugv_ranger_client=self.robot_clients[robots[0]]
                )
                target_trap = inputs[1]
                action_input = DisarmTrapGroundInput()
                action_input.trap.name = target_trap
                action_input.trap.position.x = self.database["objectives"][target_trap]["data"]["position"]["x"]
                action_input.trap.position.y = self.database["objectives"][target_trap]["data"]["position"]["y"]
                action.start(action_input)
                self.update_mission_status(i,"true")
                result = action.wait_result()
                self.update_mission_status(i,"false",action.result_to_str(result))

            elif action_name == "disarm_trap_air":
                action = DisarmTrapAir(
                    controller=self.composites_controller,
                    uav_ranger_client=self.robot_clients[robots[0]]
                )
                target_trap = inputs[1]
                action_input = DisarmTrapAirInput()
                action_input.trap.name = target_trap
                action_input.trap.position.x = self.database["objectives"][target_trap]["data"]["position"]["x"]
                action_input.trap.position.y = self.database["objectives"][target_trap]["data"]["position"]["y"]
                action.start(action_input)
                self.update_mission_status(i,"true")
                result = action.wait_result()
                self.update_mission_status(i,"false",action.result_to_str(result))

            elif action_name == "disarm_trap_combi":
                action = DisarmTrapCombined(
                    controller=self.composites_controller,
                    uav_ranger_client=self.robot_clients[robots[0]],
                    ugv_ranger_client=self.robot_clients[robots[1]]
                )
                target_trap = inputs[1]
                action_input = DisarmTrapCombinedInput()
                action_input.trap.name = target_trap
                action_input.trap.position.x = self.database["objectives"][target_trap]["data"]["position"]["x"]
                action_input.trap.position.y = self.database["objectives"][target_trap]["data"]["position"]["y"]
                action.start(action_input)
                self.update_mission_status(i,"true")
                result = action.wait_result()
                self.update_mission_status(i,"false",action.result_to_str(result))

        return True


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    ample_planning_handle = AmpleUPImplNode()
    ample_planning_handle.get_logger().info("Ample-UP Ready!")
    rclpy.spin(
        ample_planning_handle,
        executor=MultiThreadedExecutor(),
    )
    rclpy.shutdown()


if __name__ == "__main__":
    main()

