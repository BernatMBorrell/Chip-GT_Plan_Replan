#!/usr/bin/env python

from plan_factorizer.plan import FactorizationLevel, create_plan
from plan_factorizer.utils import load_module
import os
import ast
import json
import traceback
import pexpect
import rclpy
import time
import threading
import multiprocessing
from numbers import Number
from string import Template
from rclpy.signals import SignalHandlerOptions
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.publisher import Publisher
from rclpy.timer import Timer
from rclpy.task import Future
from rclpy.action import ActionServer, CancelResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.service import Service
from launch import LaunchDescription, LaunchService
from launch_ros.actions import Node as launch_Node
from ample_msgs.msg import AmpleState
from ample_msgs.srv import *
from ample_msgs.action import *
from rl_composition_py.controller import Controller
from rl_composition_py.cs_utils.init_yaml_csinputs import init_yaml_csinputs
from itertools import count

FORCE_STATIC = False
# try auspex
try:
    from auspex_msgs.srv import QueryKnowledge
    from auspex_msgs.msg import KnowledgeChange
except ImportError:
    print("Could not import auspex data, static Python database will be required.")
    FORCE_STATIC = True


def convert_numbers(d):
    if isinstance(d, dict):
        for k, v in d.items():
            if isinstance(v, dict):
                d[k] = convert_numbers(v)
            else:
                try:
                    d[k] = ast.literal_eval(v)
                except:
                    d[k] = v
    elif isinstance(d, list):
        for k, v in enumerate(d):
            if isinstance(v, (dict, list)):
                d[k] = convert_numbers(v)
            else:
                try:
                    d[k] = ast.literal_eval(v)
                except:
                    d[k] = v
    return d


def build_controller(folder: str, controller: str) -> bool:
    result = False
    try:
        child = pexpect.spawn("bash", timeout=None)
        child.sendline("cd "+folder)
        child.sendline("source /opt/ros/$ROS_DISTRO/setup.bash")
        child.sendline("colcon build --packages-select-regex "+controller)
        for line in child:
            line = line.rstrip().decode()
            print("[build]>", line)
            if "Summary:" in line:
                if not ("aborted" in line or "failed" in line):
                    result = True
                break
        child.sendline('exit')
        child.expect(pexpect.EOF)
    except KeyboardInterrupt:
        child.close(force=True)
    finally:
        return result


def generate_launch_description(controller, robots):
    # TODO: to use this, needs LaunchService to run in main thread, i.e. idk where
    # leaving for now as it should be the correct way to launch the team controller
    robot_params = ""
    for robot_name, robot_dict in robots.items():
        robot_params += f" -p {robot_name}:={robot_dict["manager"]}"
    controller_node = launch_Node(
        package=controller,
        executable=controller+".py",
        name=controller,
        parameters=[
            {"yaml_input": "/opt/jazzy_ws/src/webots_skillsets/" +
                controller+"/yaml/plan.yaml"},
            {"autorun": True},
            {"debug": True},
            robot_params,
        ]
    )
    return LaunchDescription([
        controller_node,
    ])


class OutputType:
    RL = "rl"
    JSON = "json"


class AmplePlanningNode(Node):
    """
    AMPLE node base class.
    """

    __id: count = count(0)

    # Contructor
    def __init__(self, node_name: str = "ample"):
        super().__init__(node_name=node_name)
        self.get_logger().info("Creating AMPLE node...")

        self.declare_parameter("autorun", False)
        self.declare_parameter("period", 1.0)
        self.declare_parameter("timeout", 60)
        self.declare_parameter("init_domain", "")
        self.declare_parameter("init_problem", "")
        self.declare_parameter("elementary_actions", "")
        self.declare_parameter("effect_to_result", "")
        self.declare_parameter("templated_problem", False)
        self.declare_parameter("controller", "")
        self.declare_parameter("database", "")
        self.declare_parameter("factorize", False)
        self.declare_parameter("output", OutputType.JSON)  # rl, json
        self.declare_parameter("open_gui", False)
        self.declare_parameter("assigned_team", "")

        self.__autorun = self.get_parameter(
            "autorun").get_parameter_value().bool_value
        self.__period = self.get_parameter(
            "period").get_parameter_value().double_value
        self.__timeout = self.get_parameter(
            "timeout").get_parameter_value().integer_value
        self.__timer: Timer = self.create_timer(
            timer_period_sec=self.__period, callback=self.state_callback)
        self.__database_param = self.get_parameter(
            "database").get_parameter_value().string_value
        self.__factorize = self.get_parameter(
            "factorize").get_parameter_value().bool_value
        self.__output = self.get_parameter(
            "output").get_parameter_value().string_value
        self.__controller_param = self.get_parameter(
            "controller").get_parameter_value().string_value
        self.__open_gui = self.get_parameter(
            "open_gui").get_parameter_value().bool_value
        self.__assigned_team = self.get_parameter(
            "assigned_team").get_parameter_value().string_value

        self.__database: dict = {
            "robots": {},
            "data": {
                "area": {
                    "id": ""
                }}
        }
        self.__state: AmpleState = AmpleState(state=AmpleState.WAITING_PLOBLEM)
        self.__state_pub: Publisher = self.create_publisher(
            AmpleState, "~/state", 10
        )
        self.templated_problem: bool = self.get_parameter(
            "templated_problem").get_parameter_value().bool_value
        self.__last_saved_problem: str = None

        self.domain: str = self.get_parameter(
            "init_domain").get_parameter_value().string_value
        self.problem: str = self.get_parameter(
            "init_problem").get_parameter_value().string_value
        if (file := self.get_parameter(
                "elementary_actions").get_parameter_value().string_value) != "":
            with open(file, "r") as j_f:
                self.__database["elementary_actions__as_is"] = json.load(j_f)
        if (file := self.get_parameter(
                "effect_to_result").get_parameter_value().string_value) != "":
            module = load_module(file)
            self.__database["effect_to_result"] = module.effect_to_result
            self.__database["result_to_effect"] = module.result_to_effect

        if self.__domain is not None and self.__problem is not None:
            self.__state.state = AmpleState.WAITING_DATABASE
        self.__controller: Controller = None

        # preleminary checks
        if self.__output not in [OutputType.JSON, OutputType.RL]:
            raise ValueError("Unknown output type, use: 'rl' or 'json'")
        if self.__database_param != "" and not os.path.exists(self.__database_param):
            raise ValueError(
                f"Database file not found: {self.__database_param}")
        if self.__database_param == "":
            if FORCE_STATIC is True:
                raise RuntimeError(
                    "Cannot run without static Python database. Please specify a file.")
            self.__state.state = AmpleState.WAITING_DATABASE
        if self.__controller_param == "":
            self.get_logger().info("No controller specified. AMPLE in planning-only mode.")
        else:
            self.__controller = Controller(
                path_to_json=self.__controller_param
            )

        self.__results: dict[int, dict] = {}
        self.__launch_service: LaunchService = None
        self.__controller_process: pexpect.spawn = None

        self.get_logger().info("Creating Load Problem Service")
        self.__load_problem_service: Service = self.create_service(
            LoadProblem, "~/load_problem", self.load_problem)

        self.plan_ready_event = threading.Event()

        self.get_logger().info("Creating Planning Request Action Client")
        # planning_request action for planner
        self.planning_request_act: ActionServer = ActionServer(self,
                                                               PlanningRequest,
                                                               "~/planning_request",
                                                               execute_callback=self.planning_request_cb
                                                               )
        self.execution_request_act: ActionServer = ActionServer(self,
                                                                ExecutionRequest,
                                                                "~/execution_request",
                                                                execute_callback=self.execution_request_cb,
                                                                cancel_callback=self.execution_cancel_cb
                                                                )

        # flags
        self.current_request_finished: bool = False

        # skinet-gui
        self.__gui: multiprocessing.Process = None
        if self.__open_gui:
            self.get_logger().info("Starting SkiNet GUI")
            self.start_gui()

    def start_gui(self):
        if self.__gui is not None:
            self.get_logger().info("Killing previous GUI")
            self.__gui.terminate()
        self.__gui: multiprocessing.Process = multiprocessing.Process(
            target=self.open_skinet_gui,
            args=()
        )
        self.__gui.start()

    def open_skinet_gui(self):
        self.get_logger().info("Loading SkiNet-GUI...")
        try:
            import sys
            import PyQt5.QtWidgets
            from skinet_gui.window.skinet_gui_window import SkinetGuiWindow
            app = PyQt5.QtWidgets.QApplication(sys.argv)
            self.__skinet_wnd: SkinetGuiWindow = SkinetGuiWindow(
                config_file=self.__controller_param,
                controller=self.__controller,
            )
            self.__skinet_wnd.show()
            app.exec_()
            self.get_logger().info("SkiNet-GUI closed.")
        except Exception as e:
            if isinstance(e, ImportError):
                self.get_logger().info("PyQT5 or SkiNet-GUI not installed.")
            else:
                self.get_logger().info(str(e))

    @property
    def period(self) -> float:
        return self.__period

    @property
    def timeout(self) -> int:
        return self.__timeout

    @property
    def database(self) -> dict:
        return self.__database

    @property
    def domain(self) -> str:
        return self.__domain

    @domain.setter
    def domain(self, value: str):
        if value == "":
            self.__domain = None
            self.__state.state = AmpleState.WAITING_PLOBLEM
            return
        if not isinstance(value, str):
            raise ValueError(
                f"{value} should be string, but is a {type(value)}.")
        if not os.path.exists(value):
            raise FileNotFoundError(value)
        self.get_logger().info(f"Set domain to: {value}")
        self.__domain = value

    @property
    def problem(self) -> str:
        return self.__problem

    @problem.setter
    def problem(self, value: str):
        if value == "":
            self.__problem = None
            self.__state.state = AmpleState.WAITING_PLOBLEM
            return
        if not isinstance(value, str):
            raise ValueError(
                f"{value} should be string, but is a {type(value)}.")
        if not os.path.exists(value):
            raise FileNotFoundError(value)
        self.get_logger().info(f"Set problem to: {value}")
        self.__problem = value

    @property
    def templated_problem(self) -> str:
        return self.__templated_problem

    @property
    def last_saved_problem(self) -> str:
        return self.__last_saved_problem

    @last_saved_problem.setter
    def last_saved_problem(self, value: str):
        self.__last_saved_problem = value

    @templated_problem.setter
    def templated_problem(self, value: str):
        self.__templated_problem = value

    @property
    def results(self) -> dict[int, dict]:
        return self.__results

    @property
    def assigned_team(self) -> str:
        return self.__assigned_team

    def sending_planning_request(self, request: PlanningReq.Request):
        self.get_logger().info(f"Sending planning request {request.id}")
        self.request_start_time = self.get_clock().now()
        action_request = PlanningRequest.Goal()
        action_request.id = request.id
        action_request.duration = request.duration
        action_request.planner = request.planner
        action_request.planner_parameters = request.planner_parameters
        self.current_request_goal_handle = self.planning_request_act.send_goal_async(
            action_request)
        self.current_request_goal_handle.add_done_callback(
            self.request_converged)
        self.current_request_finished = False
        self.__state.state = AmpleState.PLANNING

    # launch front planning request
    def launch_front_request(self):
        if len(self.requests):
            self.current_request = self.requests.pop(0)
            self.get_logger().debug("Current request %i" % (self.current_request.id))
            self.sending_planning_request(self.current_request)
        else:
            self.__state.state = AmpleState.IDLE

    # add planning_request
    def add_planning_request(self, request: PlanningReq.Request, response: PlanningReq.Response) -> PlanningReq.Response:
        self.requests.append(request)
        self.get_logger().debug(f"Adding request {request.id}")
        self.get_logger().debug("Requests:")
        self.get_logger().debug(self.requests)
        self.__state.state = AmpleState.PLANNING
        response.done = True
        return response

    # remove planning_request
    def remove_planning_request(self, request: PlanningReq.Request, response: PlanningReq.Response) -> PlanningReq.Response:
        if self.current_request.id == request.id and not self.current_request_finished:
            self.current_request_goal_handle.cancel()
        else:
            if request in self.requests:
                self.requests.remove(request)
        if len(self.requests) == 0:
            self.__state.state = AmpleState.IDLE
        response.done = True
        return response

    # checking if current request has converged
    def request_converged(self, future: Future):
        goal_handle = self.current_request_goal_handle.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        if goal_handle.result.id == self.current_request.id:
            self.get_logger().debug(
                f"ID {self.current_request.id} CurrentRequest")
            self.get_logger().debug(
                f"      => Finished?={goal_handle.result.finished} PlanningRequestFeedback")
            self.current_request_finished = goal_handle.result.finished
        else:
            self.current_request_finished = False

    # load problem service callback
    def load_problem(self, request: LoadProblem.Request, response: LoadProblem.Response) -> LoadProblem.Response:
        self.__state.state = AmpleState.LOADING_PROBLEM
        response.done = False
        try:
            self.domain = request.domain
            self.problem = request.problem
            if not self.__state.state == AmpleState.WAITING_DATABASE:
                self.__state.state = AmpleState.IDLE
        except:
            self.get_logger().error("Error loading problem.")
            self.__state.state = AmpleState.ERROR
        finally:
            return response

    def planning_request_cb(self, goal_handle: ServerGoalHandle) -> PlanningRequest.Result:
        id = goal_handle.request.id
        if id == 0:
            id = next(self.__id)
        self.get_logger().info(f"Received external planning request (id:{id})")

        if self.__state.state not in [AmpleState.IDLE, AmpleState.PLANNING]:
            self.get_logger().info('Ample is not ready!')
            goal_handle.abort()
            result = PlanningRequest.Result()
            result.success = False
            result.plan = "{}"
            return result

        domain = goal_handle.request.domain
        if domain == "":
            with open(self.__domain, "r") as f:
                domain = "\n".join(f.readlines())
        problem = goal_handle.request.problem
        if problem == "":
            if self.templated_problem:
                problem = self.__last_saved_problem
            else:
                with open(self.__problem, "r") as f:
                    problem = "\n".join(f.readlines())

        ###
        plan_dict = self.plan(id, domain, problem)
        ###

        result = PlanningRequest.Result()
        if plan_dict is not None:
            self.get_logger().info('Plan obtained.')
            goal_handle.succeed()
            result.success = True
            if self.__output == OutputType.JSON:
                result.plan = json.dumps(plan_dict)
                self.__results[id] = (None, plan_dict)
            elif self.__output == OutputType.RL:
                plan_rl = self.to_rl(plan_dict)
                result.plan, plan_file = self.analyze_plan(plan_rl, id)
                self.__results[id] = (plan_file, plan_dict)
        else:
            self.get_logger().info('No Plan.')
            goal_handle.abort()
            result.success = True
            result.plan = "{}"
        self.get_logger().info(f"Planning ID '{id}' finished.")
        return result

    def plan(self, id, domain, problem) -> dict | None:
        self.get_logger().info('Executing planning...')
        self.__state.state == AmpleState.PLANNING
        plan_dict = None
        try:
            plan_dict = self.run_planner(id, domain, problem)
        except Exception as e:
            traceback.print_tb(e.__traceback__)
            print(e)
        self.__state.state == AmpleState.IDLE
        return plan_dict

    def analyze_plan(self, plan_rl: str, id: int):
        plan_name = "plan_"+self.__assigned_team+str(id)
        plan_file = "/tmp/"+plan_name+".rl"
        with open(plan_file, "w") as f:
            f.write(plan_rl)
        created_cs = self.__controller._parse_rl_model([plan_file])
        self.__controller.model["include"].append(plan_file)
        self.__controller.set_mission("plan")
        if self.__gui:
            self.start_gui()
        self.get_logger().info("Running analysis...")
        results = self.__controller.analyze_mission(pp=True)
        if all(res == True for res in results["success"].values()):
            self.get_logger().info("Model-checking found no errors.")
        else:
            self.get_logger().info("Model-checking found errors:")
            target_success = list(results["success"].keys())[0]
            self.get_logger().info(
                str(results["exits"][target_success]))
        return plan_rl, plan_file

    def to_rl(self, plan_dict: dict) -> str:
        factorization = FactorizationLevel.NONE
        if self.__factorize:
            factorization = FactorizationLevel.SOFT
        self.get_logger().info("Creating plan instance for export...")
        try:
            plan = create_plan(
                plan_dict,
                database=self.__database,
                no_encoding=False,
                factorization=factorization
            )
        except:
            # try again as sometimes it crashes when creating the NFA, idk why yet...
            plan = create_plan(
                plan_dict,
                database=self.__database,
                no_encoding=False,
                factorization=factorization
            )
        self.get_logger().info("Exporting to RL...")
        plan_rl = plan.to_rl_composition(factorization)
        self.get_logger().info(plan_rl)
        return plan_rl

    def _generate_plan(self, id: int):
        self.get_logger().info("Generating controller...")
        self.__controller._parse_rl_model([self.__results[id][0]])
        self.__controller.set_mission("plan")
        self.__controller.generate_controller_py()

        self.get_logger().info("Generating inputs using database...")
        init_yaml_csinputs(
            controller=self.__controller,
            database=self.__database
        )

        self.get_logger().info("Building...")
        controller = self.__controller.name
        workspace = os.path.abspath(self.__controller.folder)
        if "workspace" in self.__controller.model.keys():
            workspace = os.path.abspath(self.__controller.model["workspace"])
        if build_controller(workspace, controller):
            self.get_logger().info("Building successful.")
        else:
            self.get_logger().error("Buidling failed, removing result...")
            self.__results.pop(id)

    def execute_controller(self) -> bool:
        controller = self.__controller.name
        robots = self.__database["robots"]
        workspace = os.path.abspath(self.__controller.folder)
        if "workspace" in self.__controller.model.keys():
            workspace = os.path.abspath(self.__controller.model["workspace"])

        result = False
        try:
            robot_params = ""
            for robot_name, robot_dict in robots.items():
                if robot_dict["kind"] != "ranger":
                    robot_params += f" -p {robot_name}:={robot_dict["manager"]}"
            self.__controller_process = pexpect.spawn("bash", timeout=None)
            self.__controller_process.sendline("cd "+workspace)
            self.__controller_process.sendline("source install/setup.bash")
            self.__controller_process.sendline(
                "ros2 run "+controller+" "+controller+".py --ros-args" +
                " -p yaml_input:="+os.path.join(self.__controller.folder, self.__controller.name, "yaml", self.__controller.mission.name+".yaml") +
                " -p autorun:=true" +
                " -p debug:=true" +
                robot_params
            )
            for line in self.__controller_process:
                line = line.rstrip().decode()
                print("[exec]>", line)
                if "Mission result:" in line:
                    if "success" in line:
                        result = True
                    break
            self.__controller_process.close(force=True)
        except Exception as e:
            if isinstance(e, KeyboardInterrupt):
                self.__controller_process.close(force=True)
            else:
                traceback.print_tb(e.__traceback__)
                print(e)
        finally:
            return result

    def execution_request_cb(self, goal_handle: ServerGoalHandle) -> ExecutionRequest.Result:
        request: ExecutionRequest.Goal = goal_handle.request
        if request.id not in self.__results.keys():
            self.get_logger().info(
                f"ID '{request.id}' not found within results.")
            goal_handle.abort()
            result = ExecutionRequest.Result()
            result.success = False
            result.message = "ID does not exist."
            return result
        self.get_logger().info(f"ID '{request.id}' found.")

        if self.__launch_service is not None:
            self.get_logger().info(f"Controller already running. Shutting down.")
            self.__controller_process.close(force=True)

        # execute
        exec_result = self.execute(request.id)

        # send result
        result = ExecutionRequest.Result()
        if exec_result is True:
            goal_handle.succeed()
            result.success = True
            result.message = "success"
        else:
            goal_handle.abort()
            result.success = False
            result.message = "failed"

        return result

    def execute(self, id) -> bool:
        # generate
        self._generate_plan(id)
        # execute controller
        result = self.execute_controller()
        # TODO (for clean launch RL of compo as mission) to use in the main thread, i.e. idk where
        # self.__launch_service = LaunchService()
        # self.__launch_service.include_launch_description(generate_launch_description(controller,robots))
        # self.__launch_service.run()
        return result

    def execution_cancel_cb(self, goal_handle: ServerGoalHandle) -> CancelResponse:
        if self.__controller_process is not None:
            self.get_logger().info(f"Shutting down current execution.")
            self.__controller_process.close(force=True)
        return CancelResponse.ACCEPT

    def run_planner(self, id: int, domain: str, problem: str) -> dict:
        raise NotImplementedError

    # Running AmplePlanning Node
    def state_callback(self):
        # running Ample Planning
        self.__state_pub.publish(self.__state)

        # checking/updating database
        if self.__database_param == "":
            if self.__state.state == AmpleState.WAITING_DATABASE:
                self.get_logger().info("No database specified, looking for AUSPEX-KNOW database...")
                self.__database_query_client = self.create_client(
                    QueryKnowledge,
                    "/auspex_know/query_knowledge"
                )

                if not self.__database_query_client.wait_for_service(timeout_sec=5):
                    raise TimeoutError("Could not reach AUSPEX-KNOW...")
                self.get_logger().info("AUSPEX-KNOW Reached!")
                self.__state.state = AmpleState.IDLE

        if self.__state.state == AmpleState.IDLE:
            self.__db_req_future_objects = self.__database_query_client.call_async(
                QueryKnowledge.Request(collection="object"))
            self.__db_req_future_objects.add_done_callback(self.update_db_cb)
            self.__db_req_future_platforms = self.__database_query_client.call_async(
                QueryKnowledge.Request(collection="platform"))
            self.__db_req_future_platforms.add_done_callback(self.update_db_cb)

            if not self.__autorun:
                self.get_logger().debug("Loaded. Waiting for planning request...")
            else:
                if self.__last_saved_problem is None:
                    self.get_logger().info(
                        "[AUTORUN] Problem not yet created.")
                    return

                current_id = next(self.__id)
                self.get_logger().info(
                    f"[AUTORUN][{current_id}] Loading planning problem...")
                # load domain and problem
                with open(self.__domain, "r") as f:
                    domain = "\n".join(f.readlines())
                if self.templated_problem:
                    problem = self.__last_saved_problem
                else:
                    with open(self.__problem, "r") as f:
                        problem = "\n".join(f.readlines())
                
                # AMPLE async dispatcher
                self.get_logger().info(f"[AUTORUN][{current_id}] Starting planner thread...")
                self.__state.state = AmpleState.PLANNING

                # 1. Reset the syncroniz\tion event so teh execution thread knows to wait for the new plan
                self.plan_ready_event.clear()

                # 2. Thread 1: The planner
                threading.Thread(
                    target=self.async_plan_worker,
                    args=(current_id,),
                    daemon=True
                ).start()

                # 3. Thread 2: The executor
                threading.Thread(
                    target=self.async_exec_worker,
                    args=(current_id,),
                    daemon=True
                ).start()

        elif self.__state.state == AmpleState.PLANNING:
            self.get_logger().debug("Planning...")
        elif self.__state.state == AmpleState.WAITING_PLOBLEM:
            self.get_logger().debug("Waiting for domain/problem...")
        elif self.__state.state == AmpleState.ERROR:
            self.get_logger().debug("Error state...")

    def update_db_cb(self, future: Future):
        result: QueryKnowledge.Response.answer = future.result().answer
        if len(result) == 0:
            self.get_logger().debug("Database collection is empty.")
            return

        result = [json.loads(x.replace("\'", "\"")) for x in result]
        result_dict = convert_numbers(result)

        if result_dict is not None:
            if future == self.__db_req_future_objects or "locations" in result_dict[0].keys():
                result_dict = result_dict[0]
                self.__database["inputs_keys"] = list(result_dict.keys())
                for key in self.__database["inputs_keys"]:
                    self.__database[key] = result_dict[key]
                self.get_logger().debug(
                    f"Inputs keys set: {str(self.__database['inputs_keys'])}")
            else:
                assigned_team_dict = None
                if self.__assigned_team != "":
                    for team_dict in result_dict:
                        if team_dict["name"] == self.__assigned_team:
                            assigned_team_dict = team_dict
                else:
                    assigned_team_dict = result_dict[0]
                if assigned_team_dict is not None:
                    self.__database["data"] = assigned_team_dict["data"]
                    self.__database["robots"] = assigned_team_dict["agents"]

            if self.templated_problem:
                self.get_logger().debug(f"Updating problem from template.")
                self.update_problem_template()

            self.get_logger().debug(f"Updated from AUSPEX-KNOW database.")
        else:
            raise RuntimeError(
                "Database is empty. Check if Valkey server is started.")

    def update_problem_template(self):
        with open(self.__problem, "r") as f:
            template = Template(f.read())
        d = {
            "locations": "",
            "robots": "",
            "objectives": "",
            "robots_types": "",
            "robots_locations": "",
            "objectives_locations": "",
            "goals": "",
        }

        for loc in self.__database["locations"].keys():
            if loc.startswith(self.__database["data"]["area"]["id"]):
                d["locations"] += f"{loc} "
        d["locations"] += "- location"
        for obj, obj_dict in self.__database["objectives"].items():
            if obj.startswith(self.__database["data"]["area"]["id"]):
                d["objectives"] += f"{obj} "
                d["objectives_locations"] += f"(t-at {obj} {obj_dict["location"]}) "
                d["goals"] += f"(safe {obj_dict["location"]}) "

        if d["objectives"] != "":
            d["objectives"] += "- trap"

        for robot_name, robot_dict in self.__database["robots"].items():
            d["robots"] += f"{robot_name} "
            d["robots_types"] += f"(r-{robot_dict["kind"]} {robot_name}) "
            d["robots_locations"] += f"(r-at {robot_name} {robot_dict["location"]}) "
        d["robots"] += "- robot"

        self.__templated_problem = "/tmp/"+self.__assigned_team+"temp_problem.pddl"
        self.__last_saved_problem = template.substitute(d)
        with open(self.__templated_problem, "w") as f:
            f.write(self.__last_saved_problem)
    
    def async_plan_worker(self, current_id):
        with open(self.__domain, "r") as f:
            domain = "\n".join(f.readlines())
        if self.templated_problem:
            problem = self.__last_saved_problem
        else:
            with open(self.__problem, "r") as f:
                problem = "\n".join(f.readlines())
        
        self.get_logger().info(f"[PLANNER THREAD] [{current_id}] Running planner...")
        plan_dict = self.plan(id=current_id, domain=domain, problem=problem)

        if plan_dict is None:
            self.get_logger().error(f"[PLANNER THREAD] [{current_id}] No plan was found.")
            self.__state.state = AmpleState.ERROR
            return
        
        self.get_logger().info(f"[PLANNER THREAD] [{current_id}] Plan obtained.")
        if self.__output == OutputType.JSON:
            self.__results[current_id] = (None, plan_dict)
        elif self.__output == OutputType.RL:
            plan_rl = self.to_rl(plan_dict)
            plan_rl, plan_file = self.analyze_plan(plan_rl, current_id)
            self.__results[current_id] = (plan_file, plan_dict)

        self.get_logger().info(f"[PLANNER THREAD] [{current_id}] Triggering plan_ready_event.")
        self.plan_ready_event.set()

    def async_exec_worker(self, current_id):
        self.get_logger().info(f"[EXECUTION THREAD] [{current_id}] Waiting for plan_ready_event...")
        self.plan_ready_event.wait()  # Pause this thread until thread 1 calls set()

        if self.__state.state == AmpleState.ERROR:
            self.get_logger().error(f"[EXECUTION THREAD] [{current_id}] Aborting execution due to planner error.")
            return

        self.get_logger().info(f"[EXECUTION THREAD] [{current_id}] Starting execution...")
        result = self.execute(current_id)
    
        self.__last_saved_problem = None
        if result is True:
            self.get_logger().info(f"[EXECUTION] [{current_id}] Plan successfully completed. Stopping AMPLE until new problem.")
            self.__state.state = AmpleState.WAITING_PLOBLEM
        else:
            self.get_logger().info(f"[EXECUTION] [{current_id}] Plan failed. Replanning!")
            self.__state.state = AmpleState.IDLE


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    ample_planning_handle = AmplePlanningNode()
    ample_planning_handle.get_logger().info("Ample Ready!")
    rclpy.spin(
        ample_planning_handle,
        executor=MultiThreadedExecutor(),
    )
    rclpy.shutdown()


if __name__ == "__main__":
    main()

