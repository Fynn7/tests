import rclpy
import time

from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.node import Node
import rcl_interfaces.msg

from ur_dashboard_msgs.msg import RobotMode
from ur_dashboard_msgs.msg import SafetyMode
from ur_dashboard_msgs.msg import ProgramState

from hmi_interfaces.msg import EStop
from hmi_interfaces.msg import ProcessStep
from hmi_interfaces.msg import RPiInputs
from hmi_interfaces.msg import ManualGuidance
from hmi_interfaces.msg import WhichLED
from ur_msgs.srv import SetIO
from std_msgs.msg import Bool
from rcl_interfaces.srv import GetParameters, SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue

from pymoveit2 import MoveIt2
# vermutlich, weil pymoveit2 mit --merge-install --symlink-install gebuildet werden soll!
from pymoveit2.robots import ur10e

from controller_manager_msgs.srv import SwitchController
from ur_dashboard_msgs.srv import GetRobotMode, GetSafetyMode, GetProgramState

from rclpy.action import ActionServer, GoalResponse, CancelResponse
# from hlc_action_interfaces.action import HLC
from hmi_cobotank_interfaces.action import HLC


class SchedulerNode(Node):
    def __init__(self):
        super().__init__('scheduler_node')
        # Variablen definieren
        self.estop_visual = False
        self.estop_robot = False
        self.error_state = False
        self.processstep = -1
        self.processstepsuccess_gui = -1
        self.processstepsuccess_rpi = -1
        self.processstepsuccess_rpi_handgriff = -1
        self.processstepsuccess_scheduler = -1
        self.confirm = 0
        self.manualguidance = False
        # "WARNING: got wrong controller_string in switch_controller()"
        self.WRONG_STRING_SWITCH_CONTROLLER = False
        # Failure while switching controllers! Stop execution
        self.FAILURE_STRING_SWITCH_CONTROLLER = False
        # 'Failed to get response: Service Robot Mode
        self.GET_SERVICE_ROBOT_MODE = False
        self.GET_SERVICE_SAFE_MODE = False  # 'Failed to get response: Service Safe Mode
        # "WARNING: got wrong pose_string in move_robot()"
        self.MOVE_ROBOT = False
        # Failed to get response Program State
        self.GET_SERVICE_PROGRAM_STATE = False
        self.SET_IO = False                             # Failed to set UR digital output
        self.PROCESSING_CALLBACK = False

        self.whichProgramrunning = True
        self.whichMode = 7
        self.whichSafeMode = 1
        self.previous_index = 0
        self.previous_manualguidance = False
        self.last_manualguidance = None

        # Sets:
        self.processstep_manualconfirm = {
            2, 3, 4, 5, 7, 8, 9, 10, 11, 12, 13, 15, 16, 17, 18, 19, 20, 23}

        # Define dictionaries mapping mode numbers to their names
        self.robot_mode_names = {
            -1: "NO_CONTROLLER",
            0: "DISCONNECTED",
            1: "CONFIRM_SAFETY",
            2: "BOOTING",
            3: "POWER_OFF",
            4: "POWER_ON",
            5: "IDLE",
            6: "BACKDRIVE",
            7: "RUNNING",
            8: "UPDATING_FIRMWARE",
        }

        self.safety_mode_names = {
            1: "NORMAL",
            2: "REDUCED",
            3: "PROTECTIVE_STOP",
            4: "RECOVERY",
            5: "SAFEGUARD_STOP",
            6: "SYSTEM_EMERGENCY_STOP",
            7: "ROBOT_EMERGENCY_STOP",
            8: "VIOLATION",
            9: "FAULT",
            10: "VALIDATE_JOINT_ID",
            11: "UNDEFINED_SAFETY_MODE",
            12: "AUTOMATIC_MODE_SAFEGUARD_STOP",
            13: "SYSTEM_THREE_POSITION_ENABLING_STOP",
        }
        # define an expandable array for error
        self.Error_Array = []

        # returen of the function in case of the error
        self.error_name = {
            1: "WRONG_STRING_SWITCH_CONTROLLER",
            2: "FAILURE_STRING_SWITCH_CONTROLLER",
            3: "GET_SERVICE_ROBOT_MODE",
            4: "GET_SERVICE_SAFE_MODE",
            5: "MOVE_ROBOT",
            6: "GET_SERVICE_PROGRAM_STATE",
            8: "SET_IO",
            10: "PROCESSING_CALLBACK",
            11: "FAILD_TO_SET_ERROR_SCALE",
            18: "FAILD_TO_GET_PROCESSSTEP_MANUALCONFIRM"
        }

        # Roboterposen:
        self.declare_parameter(
            "start_pose",
            [
                0.0,
                -2.61799,
                2.61799,
                0.0,
                0.0,
                0.0,
            ],
        )
        self.declare_parameter(
            "get_hose_pose",
            [
                0.0,
                -1.5708,
                1.5708,
                0.0,
                0.0,
                0.0,
            ],
        )
        self.declare_parameter(
            "handover_pose",
            [
                -0.0113,
                -1.9288,
                2.2109,
                -0.566,
                1.5128,
                0.0274,
            ],
        )

        # Create callback group that allows execution of callbacks in parallel without restrictions
        moveit_callback_group = ReentrantCallbackGroup()  # Callback-Group für MoveIT
        # Callback-Group für "normales zeug"
        normal_group = MutuallyExclusiveCallbackGroup()
        # Callback-Group für EStop und alles bzgl. Safety
        safe_group = MutuallyExclusiveCallbackGroup()

        # Create MoveIt 2 interface
        self.moveit2 = MoveIt2(
            node=self,
            joint_names=ur10e.joint_names(),
            base_link_name=ur10e.base_link_name(),
            end_effector_name=ur10e.end_effector_name(),
            group_name=ur10e.MOVE_GROUP_ARM,
            callback_group=moveit_callback_group,
        )

        # EStop-Visual Publisher:
        self.estop_visual_publisher_ = self.create_publisher(
            EStop, 'EStop_Visual', 10, callback_group=safe_group)

        # ProcessStep Publisher:
        self.processstep_publisher_ = self.create_publisher(
            ProcessStep, 'ProcessStep', 10, callback_group=normal_group)

        # ProcessStep Subscriber:
        self.subscrProcessStep = self.create_subscription(
            ProcessStep, 'ProcessStep', self.processstep_callback, 10, callback_group=normal_group)
        self.subscrProcessStep  # prevent unused variable warning

        # ProcessStepSuccess Publisher:
        self.processstepsuccess_publisher_ = self.create_publisher(
            ProcessStep, 'ProcessStepSuccess', 10, callback_group=normal_group)

        # ProcessStepSuccess Subscriber:
        self.subscrProcessStepSuccess = self.create_subscription(
            ProcessStep, 'ProcessStepSuccess', self.processstepsuccess_callback, 10, callback_group=normal_group)
        self.subscrProcessStepSuccess  # prevent unused variable warning

        # ManualGuidance Subscriber:
        self.subscrManualGuidance = self.create_subscription(
            ManualGuidance, 'ManualGuidance', self.manualguidance_callback, 10, callback_group=safe_group)
        self.subscrManualGuidance  # prevent unused variable warning

        # RPi_Inputs Subscriber:
        self.subscrRPi_Inputs = self.create_subscription(
            RPiInputs, 'RPiInputs', self.rpiinputs_callback, 10, callback_group=safe_group)
        self.subscrRPi_Inputs  # prevent unused variable warning

        # EStop_RPi Subscriber:
        self.subscrEStop_RPi = self.create_subscription(
            EStop, 'EStop_RPi', self.estop_rpi_callback, 10, callback_group=safe_group)
        self.subscrEStop_RPi  # prevent unused variable warning

        # Service-client für LEDs, Subscription zu robot mode, safe mode und programmrunning
        self.Set_IO_Survice = '/io_and_status_controller/set_io'
        self.set_io_client = self.create_client(SetIO, self.Set_IO_Survice)
        while not self.set_io_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for %s service...' % self.Set_IO_Survice)

        # Client für Service-Anfrage an UR_Driver
        self.Get_Switch_client = self.create_client(
            SwitchController, '/controller_manager/switch_controller', callback_group=moveit_callback_group)
        while not self.Get_Switch_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.request_switch_Controller = SwitchController.Request()

        # Call the service get root mode
        self.Get_Robot_Mode = '/dashboard_client/get_robot_mode'
        self.get_Robot_Mode_client = self.create_client(
            GetRobotMode, self.Get_Robot_Mode, callback_group=moveit_callback_group)
        while not self.get_Robot_Mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for %s service...' % self.Get_Robot_Mode)
        self.request_Robot_Mode = GetRobotMode.Request()

        # Call the service get safe mode
        self.Get_Safe_Mode = '/dashboard_client/get_safety_mode'
        self.get_Safe_Mode_client = self.create_client(
            GetSafetyMode, self.Get_Safe_Mode, callback_group=moveit_callback_group)
        while not self.get_Safe_Mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for %s service...' % self.Get_Safe_Mode)
        self.request_Safe_Mode = GetSafetyMode.Request()

        # Call the service get pragram state
        self.Get_Program_State = '/dashboard_client/program_state'
        self.get_Program_State_client = self.create_client(
            GetProgramState, self.Get_Program_State, callback_group=moveit_callback_group)
        while not self.get_Program_State_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for %s service...' % self.Get_Program_State)
        self.request_Program_State = GetProgramState.Request()

        # /cartesian_force_controller/get_parameters rcl_interfaces/srv/GetParameters
        self.get_Parameters = '/cartesian_force_controller/get_parameters'
        self.get_Parameters_client = self.create_client(
            GetParameters, self.get_Parameters, callback_group=moveit_callback_group)
        while not self.get_Parameters_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for %s service...' % self.get_Parameters)
        self.request_Parameters = GetParameters.Request()

        self.setParameter_client = self.create_client(
            SetParameters, '/cartesian_force_controller/set_parameters')  # is it just get_parameters
        while not self.setParameter_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service...')
        self.setParameter_request = SetParameters.Request()

        # Service-Client for setting error scale to server '/cartesian_force_controller/get_parameters'
        self.set_error_scale_server_name = '/cartesian_force_controller/set_parameters'  # server name
        self.set_error_scale_client = self.create_client(
            SetParameters, self.set_error_scale_server_name)  # !! brauchen wir hier einen callback_group?
        while not self.set_error_scale_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for %s service...' %
                                   self.set_error_scale_server_name)
        self.request_error_scale = SetParameters.Request()

        # EStop Timer
        estop_timer_period = 0.1  # seconds
        # TODO: Frequenz deutlich erhöhen!!
        self.estop_timer = self.create_timer(
            estop_timer_period, self.estop_timer_callback, callback_group=safe_group)

        # Which LED
        self.BlueLEDsubscription = self.create_subscription(
            WhichLED, 'whichLED', self.processstep_callback, 10)
        self.BlueLEDsubscription  # prevent unused variable warning

        # safety mode
        self.safe_mode_subscription = self.create_subscription(
            SafetyMode, '/io_and_status_controller/safety_mode', self.safe_mode_callback, 10)
        self.safe_mode_subscription  # prevent unused variable warning

        # Robot Mode
        self.robot_mode_subscription = self.create_subscription(
            RobotMode, '/io_and_status_controller/robot_mode', self.robot_mode_callback, 10)
        self.robot_mode_subscription  # prevent unused variable warning

        # Program State
        self.robot_program_running_subscription = self.create_subscription(
            Bool, '/io_and_status_controller/robot_program_running', self.robot_programrunning_callback, 10)
        self.robot_program_running_subscription  # prevent unused variable warning

        # action server for processsteps: 0-3,4-5,6,7-20,21,22,23

        self.hlc_action_server = ActionServer(
            self,
            HLC,
            'hlc_action',
            execute_callback=self.hlc_action_result_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )
        assert self.hlc_action_server
        self.get_logger().info(
            f"server successfully created at {self.hlc_action_server}")

        self.process_succeed = True

    def goal_callback(self, goal_request):
        '''
        Check if the goal is valid
        '''
        if goal_request.function_id not in range(50, 57):  # 50-56 sind die gültigen Funktion IDs für die HMI
            self.get_logger().error("Received invalid goal")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def hlc_action_result_callback(self, goal_handle):
        '''
        Callback function for the action server
        '''
        FP_MAPPER = {
            50: 0,
            51: 4,
            52: 6,
            53: 7,
            54: 21,
            55: 22,
            56: 23
        }
        feedback_msg = HLC.Feedback()
        feedback_msg.finished_at = -1
        requested_processstep = FP_MAPPER[goal_handle.request.function_id]

        def get_result(success: bool):
            result = HLC.Result()
            result.success = success
            return result

        def publish_processstep():
            msg = ProcessStep()
            msg.origin = "SchedulerNode"
            msg.processstep = requested_processstep
            self.processstep_publisher_.publish(msg)

        def publish_success():
            msg = ProcessStep()
            msg.origin = "SchedulerNode"
            msg.processstep = self.processstep
            self.processstepsuccess_publisher_.publish(msg)

        while requested_processstep not in {3, 5, 6, 20, 21, 22, 23}:
            self.processstep = requested_processstep
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info("Goal successfully canceled from client.")
                return get_result(success=False)

            if self.process_succeed:
                publish_processstep()
                self.process_succeed = False
            else:
                self.get_logger().info("Waiting for the result of the process step...")
                time.sleep(1)
                continue
            time.sleep(1)
            if self.process_succeed:
                publish_success()
                requested_processstep += 1

        goal_handle.succeed()
        self.get_logger().info("Goal succeeded")

        return get_result(success=True)

    def cancel_callback(self, goal_handle):
        '''
        Callback function for the cancel request
        '''
        self.get_logger().info("Canceling goal")
        return CancelResponse.ACCEPT

    def Get_Service_Robot_Mode(self):
        a = 0  # Random Varible, loop this while untill there is a response from the service
        while a != 1:  # Solange der Robot_mode nicht gleich 1 ist:
            response = self.get_Robot_Mode_client.call(self.request_Robot_Mode)
            time.sleep(1)
            self.get_logger().info('Received response: %s' % response.answer)
            # Process the response here
            if response.answer == 'Robotmode: RUNNING':
                a = 1  # got a response the value of a is now 1 no longer loop
            elif a != 1:  # still no response? keep looping
                self.get_logger().info('Waiting 1sec before asking Robot Mode Service again.')
                time.sleep(1)
                # Warte 2 Sekunden
            else:
                self.get_logger().error('Failed to get Robot Mode response')
                return 3

    def Get_Service_SAFE_Mode(self):
        a = 0  # Random Varible, loop this while untill there is a response from the service
        while a != 1:  # Solange der Robot_mode nicht gleich 1 ist:
            response = self.get_Safe_Mode_client.call(self.request_Safe_Mode)
            time.sleep(1)
            self.get_logger().info('Received response: %s' % response.answer)
            # Process the response here
            if response.answer == 'Safetymode: NORMAL':
                a = 1  # got a response the value of a is now 1 no longer loop
            elif a != 1:  # still no response? keep looping
                self.get_logger().info('Waiting 1sec before asking Safe Mode Service again.')
            else:
                self.get_logger().error('Failed to get Robot Mode response')
                return 4

    def Get_Service_Program_State(self):
        a = 0  # Random Varible, loop this while untill there is a response from the service
        while a != 1:  # Solange der Robot_mode nicht gleich 1 ist:
            response = self.get_Program_State_client.call(
                self.request_Program_State)
            time.sleep(1)

            self.get_logger().info('Received response: %s' % response.answer)
            # Process the response here
            if response.answer == 'PLAYING HMI_PC.urp' or response.answer == 'PLAYING HMI_Notebook.urp':
                a = 1  # got a response the value of a is now 1 no longer loop
                # return 6
            elif a != 1:  # still no response? keep looping
                self.get_logger().info('Waiting 1sec before asking Program State Service again.')
                # Warte 2 Sekunden
            else:
                self.get_logger().error('Failed to get Progmram State response')
                return 6

    def set_error_scale(self, error_scale: float) -> int:
        '''
        Get error scale from manualguidance callback 
        and set it to the parameter server '/cartesian_force_controller/get_parameters'
        '''
        request = self.request_error_scale

        parameter = rcl_interfaces.msg.Parameter(
            name='solver.error_scale',
            value=rcl_interfaces.msg.ParameterValue(
                type=3, double_value=error_scale)
        )

        request.parameters = [parameter]  # all parameters into a list
        response = self.set_error_scale_client.call(request)

        self.get_logger().info('Successfully set error scale to %f' % error_scale)

        if response is not None:
            pass
        else:
            return 11

    # Callback functiion for RobotMode
    def robot_mode_callback(self, msg: RobotMode):
        # self.get_logger().info('Received message from Robot Mode topic: %s ' % msg.mode)
        # define a variable for message name
        robot_mode_number = msg.mode
        # Check if the mode numbers are valid and print the corresponding mode names
        if robot_mode_number in self.robot_mode_names:
            self.get_logger().info(
                f"Received message from topic Robot Mode: {self.robot_mode_names[robot_mode_number]}")
        self.whichMode = msg.mode

    # Callback function for SafetyMode
    def safe_mode_callback(self, msg: SafetyMode):
        # self.get_logger().info('Received message from robot_mode topic: %s' % msg.mode)
        # define a variable
        safety_mode_number = msg.mode
        if safety_mode_number in self.safety_mode_names:
            self.get_logger().info(
                f"Received message from topic Safe Mode: {self.safety_mode_names[safety_mode_number]}")
        self.whichSafeMode = msg.mode

    # callback to ProgramState
    def robot_programrunning_callback(self, msg: Bool):
        self.get_logger().info('Received message from ProgramState topic: %s' % msg.data)
        self.whichProgramrunning = msg.data

    def switch_controller(self, controller_string):
        self.req = self.request_switch_Controller
        if controller_string == 'cartesian_force_controller':
            self.req.activate_controllers = ['cartesian_force_controller']
            self.req.deactivate_controllers = [
                'scaled_joint_trajectory_controller']
        elif controller_string == 'scaled_joint_trajectory_controller':
            self.req.activate_controllers = [
                'scaled_joint_trajectory_controller']
            self.req.deactivate_controllers = ['cartesian_force_controller']
        else:
            self.get_logger().warning("WARNING: got wrong controller_string in switch_controller()")
            return -1
        self.req.strictness = 2
        self.req.activate_asap = True
        self.req.timeout.sec = 10
        self.req.timeout.nanosec = 0

        response = self.Get_Switch_client.call(self.req)
        if response.ok:
            # TODO: Dieser Logger-Info wird nicht ins Terminal geschrieben? Warum??
            self.get_logger().info('Successfully switched controllers.')
            return None
        else:
            self.get_logger().error('Failure while switching controllers! Stop execution')
        #    self.Error_Array.append(2)
            # return 2

    def move_robot(self, pose_string):

        # Scale down velocity and acceleration of joints (percentage of maximum)
        self.moveit2.max_velocity = 0.2
        self.moveit2.max_acceleration = 0.2

        # Get parameter
        if pose_string == 'start_pose':
            joint_positions = (self.get_parameter(
                "start_pose").get_parameter_value().double_array_value)
        elif pose_string == 'get_hose_pose':
            joint_positions = (self.get_parameter(
                "get_hose_pose").get_parameter_value().double_array_value)
        elif pose_string == 'handover_pose':
            joint_positions = (self.get_parameter(
                "handover_pose").get_parameter_value().double_array_value)
        else:
            self.get_logger().warning("WARNING: got wrong pose_string in move_robot()")
        #    self.Error_Array.append(5)
            return 5

        # Move to joint configuration
        self.get_logger().info(
            f"Moving to {{joint_positions: {list(joint_positions)}}}")
        self.moveit2.move_to_configuration(joint_positions)
        self.moveit2.wait_until_executed()
        self.get_logger().info("Movement finished")
        # return 0

    def estop_timer_callback(self):

        if ((self.whichSafeMode == 1 or self.whichSafeMode == 2) and self.whichMode == 7 and self.whichProgramrunning == True):
            self.estop_robot = False
        else:
            self.estop_robot = True

        # self.get_logger().info('EStop-States: RPi: "%s", Robot: "%d", Visual: "%d".' % (self.estop_rpi, self.estop_robot, self.estop_visual))
        if ((self.estop_robot == True or self.error_state == True) and self.estop_visual == False):
            # Ein Not-Halt wurde gedrückt, aber noch nicht behandelt!!

            # TODO:
            # Bewegung des Roboter stoppen: error_scale = 0 oder Programmausführung mit moveit abbrechen! --> Gibt es diese Funktion??

            self.estop_visual = True
            msg = EStop()
            msg.origin = "SchedulerNode"
            msg.estop = self.estop_visual
            # msg.wrong_string_switch_controller = True

            for error_name in self.Which_Error:

                self.get_logger().info('Publishing EStop-Visual: "%s".' % error_name)
                error_name_s = print(error_name.islower())
                self.get_logger().info('Publishing EStop-Visual: "%s".' % error_name_s)

                setattr(msg, 'error_name_s', True)

                self.get_logger().info('Publishing EStop-Visual: "%s".' %
                                       msg.wrong_string_switch_controller)
                self.estop_visual_publisher_.publish(msg)
                self.get_logger().info('Publishing EStop-Visual: "%s".' % msg.estop)

        elif (self.estop_robot == False and self.estop_visual == True):
            # Beide Not-Halt sind wieder raus, die Änderung wurde aber noch nicht an GUI und Roboter weitergeleitet
            pass
            # TO def set_ur_digital_output(self, index: int , value: bool):DO:
            # Bewegung des Roboters fortsetzen
            self.estop_visual = False
            msg = EStop()
            msg.origin = "SchedulerNode"
            msg.estop = self.estop_visual
            self.estop_visual_publisher_.publish(msg)
            self.get_logger().info('Publishing EStop-Visual: "%s".' % msg.estop)

    # function setting the IO on or offget_parameter_value
    # Helper fun line 397 tion to set UR digital output
    # We are turnign LED on one two or three,..., turn the prev. ones off

    def set_ur_digital_output(self, index: int, value: bool):

        request = SetIO.Request()
        request.fun = SetIO.Request.FUN_SET_DIGITAL_OUT
        request.pin = index

        # Turn off the previous LED if it was set
        if self.previous_index is not None:
            request.pin = self.previous_index
            request.state = 1.0
            self.set_io_client.call_async(request).result()

        request.pin = index
        if value == True:
            request.state = float(0)
        elif value == False:
            request.state = float(1)

        # Update the previous index
        self.previous_index = index

        future = self.set_io_client.call(request)
        # self.get_logger().error(f'UNknown error setting error scale parameter: {future}')

        time.sleep(1)

        # if the function work do nothing if not return 8
        if future.success == True:
            pass

        else:
            return 8

    # get processstep_manualconfirm from RPI
    def Get_Parameters(self, name: str) -> int:
        request = self.request_Parameters
        request.names = [name]
        response = self.get_Parameters_client.call(request)
        self.get_logger().info(f'scheduler node got parameter: {response}')
        # if the function works return none otherwise return 18
        if response is not None:
            return None
        else:
            return 18

    def estop_rpi_callback(self, msg: EStop):
        # self.get_logger().info('EStop_RPi: I heard from "%s": "%d".' % (msg.origin, msg.estop))
        self.estop_rpi = msg.estop

    def processstep_callback(self, msg: ProcessStep):
        self.get_logger().info('Starting ProcessStep: "%d".' % msg.processstep)
        self.processstep = msg.processstep

        if self.processstep == 0:

            # TODO:

            # Abfrage, ob Roboterzustände richtig: SafeMode, RobotMode und Programmrunning
            # Falls noch nicht richtig, dann Meldung im Terminal und nach 2 sek erneut abfragen

            self.Error_Array.append(self.Get_Service_SAFE_Mode())
            self.get_logger().info('ProcessStep: I heard from Safe Mode, it is safe.')

            self.Error_Array.append(self.Get_Service_Robot_Mode())
            self.get_logger().info('ProcessStep: I heard from Robot Mode, it is running.')

            self.Error_Array.append(self.Get_Service_Program_State())
            self.get_logger().info('ProcessStep: I heard from Program State, it is playing.')

            self.Error_Array.append(
                self.Get_Parameters('processstep_manualconfirm'))
            self.get_logger().info(
                'ProcessStep: I heard from Get_Parameters, the parameters from RPi_Node')

            # Bei Meustart nach Fehler und deaktiviertem Trajectory controller müsste her erstmal der Controller gewechselt werden!!
            # Hierzu muss aber erstmal erkannt werden, dass der falsche Controller aktiv ist. Diese Abfrage hier einfügen?
            # self.Error_Array.append(self.switch_controller("scaled_joint_trajectory_controller"))

            self.Error_Array.append(self.move_robot("start_pose"))
            self.get_logger().info('Moving Robot to start pose.')

            # Warnleuchte am UR auf "Kollaboration"
            self.Error_Array.append(self.set_ur_digital_output(0, True))

        elif self.processstep == 1:
            # Warnleuchte am UR auf "Kollaboration"
            self.Error_Array.append(self.set_ur_digital_output(0, True))
        elif self.processstep == 4:
            self.Error_Array.append(self.move_robot("get_hose_pose"))
            time.sleep(3)
            self.Error_Array.append(self.move_robot("start_pose"))

        elif self.processstep == 5:
            # Warnleuchte am UR auf "Automatisch"
            self.Error_Array.append(self.set_ur_digital_output(1, True))

        elif self.processstep == 6:
            self.Error_Array.append(self.move_robot("handover_pose"))
            self.Error_Array.append(
                self.switch_controller("cartesian_force_controller"))

        elif self.processstep == 7:
            # DANN Warnleuchte am UR auf "Kollaboration"
            self.Error_Array.append(self.set_ur_digital_output(0, True))

        elif self.processstep == 20:
            # Warnleuchte am UR auf "Automatisch"
            self.Error_Array.append(self.switch_controller(
                "scaled_joint_trajectory_controller"))
            self.Error_Array.append(self.set_ur_digital_output(1, True))

        elif self.processstep == 21:
            self.Error_Array.append(self.move_robot("start_pose"))

        elif self.processstep == 22:
            self.Error_Array.append(self.move_robot("get_hose_pose"))
            time.sleep(3)
            self.Error_Array.append(self.move_robot("start_pose"))

        # check the error_array for errors
        for error_value in self.Error_Array:
            if error_value == None:
                Success = True

            else:  # publich auf Error topic und sagt welche problem
                # self.get_logger().info(f"Received message from topic error Mode: {self.Error_Array}")
                Success = False
                self.get_logger().error('ERROR: Failure in processstep_callback!')
                # There is an error in  processstep callback add 10
                self.Error_Array.append(10)

                # self.get_logger().info(f"Received message from topic error Mode: {self.Error_Array}")

                self.get_logger().info(
                    f"Received message from topic error Mode: {self.error_name[error_value]}")
                self.Error_Array = []  # empty the Error_Array

        if Success == True:
            # Publish ProcessStepSuccess
            msgsuccess = ProcessStep()
            msgsuccess.origin = "SchedulerNode"
            msgsuccess.processstep = self.processstep
            self.processstepsuccess_publisher_.publish(msgsuccess)

            self.process_succeed = True # notify the action server globally that the process has succeeded

    def processstepsuccess_callback(self, msg: ProcessStep):
        # self.get_logger().info('ProcessStepSuccess: I heard from "%s": "%d".' % (msg.origin, msg.processstep))

        if msg.origin == "GUINode":
            self.processstepsuccess_gui = msg.processstep
            self.get_logger().info('ProcessStepSuccess_GUI: "%d".' %
                                   self.processstepsuccess_gui)
        elif msg.origin == "RPiNode":
            self.processstepsuccess_rpi = msg.processstep
            self.get_logger().info('ProcessStepSuccess_RPi: "%d".' %
                                   self.processstepsuccess_rpi)
        elif msg.origin == "SchedulerNode":
            self.processstepsuccess_scheduler = msg.processstep
            self.get_logger().info('ProcessStepSuccess_Scheduler: "%d".' %
                                   self.processstepsuccess_scheduler)

        if (self.processstep == self.processstepsuccess_gui == self.processstepsuccess_rpi == self.processstepsuccess_scheduler) and (self.processstep not in self.processstep_manualconfirm):
            if self.processstep == 14:
                time.sleep(5)
                self.get_logger().info('Processstep 14: Waiting 5 seconds.')
            if self.processstep == 21:
                time.sleep(5)
                self.get_logger().info('Processstep 21: Waiting 5 seconds.')
            # self.processstep = self.processstep + 1

            # # neuen Prozessschritt publishen
            # msg_processtep = ProcessStep()
            # msg_processtep.origin = "SchedulerNode"
            # msg_processtep.processstep = self.processstep
            # self.processstep_publisher_.publish(msg_processtep)
            # self.get_logger().info('All Nodes finished. Automatically starting Processstep "%s".' %
            #                        msg_processtep.processstep)
            self.process_succeed = True

    def manualguidance_callback(self, msg: ManualGuidance):
        '''
        frequently called
        '''
       # self.get_logger().info('ManualGuidance: I heard from "%s": "%s".' % (msg.origin, msg.manualguidance))
        self.manualguidance = msg.manualguidance

        if self.manualguidance:
            error_scale = 0.3
        else:
            error_scale = 0.0  # Das Roboter steht still

        # Only calls the function when manualguidance is changed
        if self.last_manualguidance != self.manualguidance:
            self.set_error_scale(error_scale)
        self.last_manualguidance = self.manualguidance

        # Error_Scale ist ein ROS-Parameter des cartesian_force_controllers! --> https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Using-Parameters-In-A-Class-Python.html
        # Dieser muss also hier mit self.set_parameters gesetzt werden!!

    def rpiinputs_callback(self, msg: RPiInputs):
        # self.get_logger().info('RPiInputs: I heard from "%s": Confirm:"%s", Button 1:"%s", Button 2:"%s", Button 3:"%s", Button 4:"%s", Button 5:"%s", Button 6:"%s", Panel_Inserted:"%s".' % (msg.origin, msg.confirm, msg.button_1, msg.button_2, msg.button_3, msg.button_4, msg.button_5, msg.button_6, msg.panel_inserted))
        self.confirm = msg.confirm

        if self.confirm:

            self.get_logger().info('User confirmed with Button')

            if (self.processstep == self.processstepsuccess_gui == self.processstepsuccess_rpi == self.processstepsuccess_rpi_handgriff == self.processstepsuccess_scheduler) and (self.processstep in self.processstep_manualconfirm):

                self.processstep = self.processstep + 1
                # Am Ende des Prozesses von vorne beginnen:
                if self.processstep == 23:
                    self.processstep = 1

                # neuen Prozessschritt publishen
                msg_processtep = ProcessStep()
                msg_processtep.origin = "SchedulerNode"
                msg_processtep.processstep = self.processstep
                self.processstep_publisher_.publish(msg_processtep)
                # self.get_logger().info('Publishing Processstep: "%s".' % msg_processtep.processstep)

            elif self.processstep == -1:
                # Erstmaliger Prozessstart
                # Prozessschritt auf 0 setzen und publishen

                # Check des Roboterzustandes und bewegung auf Startpose erfolgt in Processstep Callback
                self.processstep = 0

                # neuen Prozessschritt publishen
                msg_processtep = ProcessStep()
                msg_processtep.origin = "SchedulerNode"
                msg_processtep.processstep = self.processstep
                self.processstep_publisher_.publish(msg_processtep)
                # self.get_logger().info('Publishing Processstep: "%s".' % msg_processtep.processstep)


def main(args=None):
    rclpy.init(args=args)

    scheduler_node = SchedulerNode()

    executor = MultiThreadedExecutor(3)
    executor.add_node(scheduler_node)

# do we need one error here?
    try:
        scheduler_node.get_logger().info('Beginning HMI-Testcase, end with CTRL-C')
        executor.spin()
    except KeyboardInterrupt:
        scheduler_node.get_logger().info('KeyboardInterrupt, shutting down.\n')

    scheduler_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
