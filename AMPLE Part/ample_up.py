#!/usr/bin/env python

import rclpy
import json
from rclpy.executors import MultiThreadedExecutor
from rclpy.signals import SignalHandlerOptions
from .ample import AmplePlanningNode

from unified_planning.shortcuts import *
from unified_planning.io import PDDLReader
from unified_planning.engines.results import PlanGenerationResult, PlanGenerationResultStatus
from unified_planning.plans import SequentialPlan
from pyparsing.exceptions import ParseBaseException

up.shortcuts.get_environment().credits_stream = None


class AmpleUPPlanningNode(AmplePlanningNode):
    """
    AMPLE node for calling planners from Unified-Planning.
    """

    # Contructor
    def __init__(self, node_name: str = "ample"):
        super().__init__(node_name=node_name)
        self.declare_parameter("planner", "")
        planner = self.get_parameter(
            "planner").get_parameter_value().string_value
        self.__up_planner = planner if planner != "" else None

    def format_action_for_json(self, action):
        action = str(action)
        action = action.replace("(", "[").replace(")", "]").replace(" ", "")
        return action

    def _convert_to_dict(self, plan: SequentialPlan):
        plan_dict = {}
        for i, action in enumerate(plan.actions):
            plan_dict[str(i)] = {
                "Task": self.format_action_for_json(action),
                "Method": "ε",
                "Branches": [i+1]
            }
        return plan_dict

    def run_planner(self, id: int, domain: str, problem: str):
        try:
            up_problem = PDDLReader().parse_problem_string(
                domain_str=domain, problem_str=problem)
        except Exception as e:
            self.get_logger().info("Error while parsing: " + str(e))
            return None

        self.portfolio = {}
        optimistic_plan_dict = None

        with OneshotPlanner(name=self.__up_planner, problem_kind=up_problem.kind) as planner:
            self.get_logger().info(f"[{planner.name}] Running planner on optimistic problem...")
            result: PlanGenerationResult = planner.solve(up_problem)
            self.get_logger().info(f"[{planner.name}] Planner finished with status: {result.status}")

            target_trap = None

            if result.status == PlanGenerationResultStatus.SOLVED_SATISFICING:
                optimistic_plan_dict = self._convert_to_dict(result.plan)
                self.get_logger().info(f"[{planner.name}] Optimistic plan found with {len(result.plan.actions)} actions.")
                
                for action in result.plan.actions:
                    if action.action.name == "detect_trap":
                        target_trap = str(action.actual_parameters[2])
                        self.get_logger().info(f"[{planner.name}] Next targeted trap is {target_trap}. Focusing portfolio here.")
                        break
            else:
                self.get_logger().info(f"[{planner.name}] No optimistic plan found. Execution aborted.")
                return None
            
            if target_trap:
                traps = ["t-air", "t-ground", "t-combi"]
                for t in traps:
                    self.get_logger().info(f"[{planner.name}] Generating portfolio for specific target: {target_trap} as {t}...")

                    search_str = f"(t-combi {target_trap})"
                    replace_str = f"({t} {target_trap})"
                    
                    trap_problem_str = problem.replace(search_str, replace_str)
                    
                    try:
                        trap_up_problem = PDDLReader().parse_problem_string(domain_str=domain, problem_str=trap_problem_str)
                        trap_result = planner.solve(trap_up_problem)

                        if trap_result.status == PlanGenerationResultStatus.SOLVED_SATISFICING:
                            self.portfolio[t] = self._convert_to_dict(trap_result.plan)
                            self.get_logger().info(f"[{planner.name}] Portfolio plan for {t} cached successfully.")
                        else:
                            self.get_logger().info(f"[{planner.name}] No valid plan found for trap scenario {t}.")
                    
                    except Exception as e:
                        self.get_logger().warning(f"[{planner.name}] Exception during portfolio generation for {t}: " + str(e))
            else:
                self.get_logger().info(f"[{planner.name}] No 'detect_trap' actions found in plan. Skipping portfolio.")

        return optimistic_plan_dict
def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    ample_planning_handle = AmpleUPPlanningNode()
    ample_planning_handle.get_logger().info(
        "AmpleUP (Ample x Unified-Planning) Ready!")
    rclpy.spin(
        ample_planning_handle,
        executor=MultiThreadedExecutor(),
    )
    rclpy.shutdown()


if __name__ == "__main__":
    main()
