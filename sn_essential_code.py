class rclpy:
    ...
import time
class Node:
    ...
class ActionServer:
    ...
class HLC:
    ...
class GoalResponse:
    ...
class CancelResponse:
    ...
class ProcessStep:
    ...

class SchedulerNode(Node):

    def __init__(self):
        super().__init__('scheduler_node')

        self.hlc_action_server = ActionServer(
            self,
            HLC,
            'hlc_action',
            execute_callback=self.hlc_action_result_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

        self.process_succeed=False
        self.running_step=False

    def goal_callback(self, goal_request):
        '''
        Check if the goal is valid
        '''
        if goal_request.function_id not in range(50,57): # 50-56 sind die gültigen Funktion IDs für die HMI
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
        self.processstep = FP_MAPPER[goal_handle.request.function_id]

        def get_result(success: bool):
            result = HLC.Result()
            result.success = success
            return result

        def publish_processstep():
            msg = ProcessStep()
            msg.origin = "SchedulerNode"
            msg.processstep = self.processstep
            self.processstep_publisher_.publish(msg)

        def publish_success():
            msg = ProcessStep()
            msg.origin = "SchedulerNode"
            msg.processstep = self.processstep
            self.processstepsuccess_publisher_.publish(msg)

        # Endzustände
        while self.processstep not in {3, 5, 6, 20, 21, 22, 23}:
            time.sleep(1)

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info("Goal successfully canceled from client.")
                self.process_succeed = False
                self.running_step = False
                return get_result(success=False)

            if not self.process_succeed and not self.running_step:
                self.get_logger().info(
                    f"Running processstep \`{self.processstep}\`.")
                publish_processstep()


            elif not self.process_succeed and self.running_step:
                self.get_logger().info(
                    f"Loop again till the previous step \`{self.processstep-1}\` is done.")
                continue

            elif self.process_succeed and not self.running_step:
                publish_success()
                self.get_logger().info(
                    f"Step \`{self.processstep}\` is done.")

            else:
                self.get_logger().info(
                    f"Error state at step \`{self.processstep}\`.")
                return get_result(success=False)

        goal_handle.succeed()
        self.get_logger().info("Goal succeeded")

        return get_result(success=True)


    def cancel_callback(self, goal_handle):
        '''
        Callback function for the cancel request
        '''
        self.get_logger().info("Canceling goal")
        return CancelResponse.ACCEPT

    def processstep_callback(self, msg: ProcessStep):
        Success = False
        self.get_logger().info('Starting ProcessStep: "%d".' % msg.processstep)
        self.processstep = msg.processstep

        if self.processstep == 0:

            # TODO:

            # Abfrage, ob Roboterzustände richtig: SafeMode, RobotMode und Programmrunning
            # Falls noch nicht richtig, dann Meldung im Terminal und nach 2 sek erneut abfragen

            self.Error_Array.append(self.Get_Service_SAFE_Mode())
            self.get_logger().info('ProcessStep: I heard from Safe Mode, it is safe.' )

            self.Error_Array.append(self.Get_Service_Robot_Mode())
            self.get_logger().info('ProcessStep: I heard from Robot Mode, it is running.' )

            self.Error_Array.append(self.Get_Service_Program_State())
            self.get_logger().info('ProcessStep: I heard from Program State, it is playing.' )

            self.Error_Array.append(self.Get_Parameters('processstep_manualconfirm'))
            self.get_logger().info('ProcessStep: I heard from Get_Parameters, the parameters from RPi_Node' )

            # Bei Meustart nach Fehler und deaktiviertem Trajectory controller müsste her erstmal der Controller gewechselt werden!!
            # Hierzu muss aber erstmal erkannt werden, dass der falsche Controller aktiv ist. Diese Abfrage hier einfügen?
            #self.Error_Array.append(self.switch_controller("scaled_joint_trajectory_controller"))

            self.Error_Array.append(self.move_robot("start_pose"))
            self.get_logger().info('Moving Robot to start pose.' )

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
            self.Error_Array.append(self.switch_controller("cartesian_force_controller"))

        elif self.processstep == 7:
            # DANN Warnleuchte am UR auf "Kollaboration"
            self.Error_Array.append(self.set_ur_digital_output(0, True))

        elif self.processstep == 20:
            # Warnleuchte am UR auf "Automatisch"
            self.Error_Array.append(self.switch_controller("scaled_joint_trajectory_controller"))
            self.Error_Array.append(self.set_ur_digital_output(1, True))

        elif self.processstep == 21:
            self.Error_Array.append(self.move_robot("start_pose"))

        elif self.processstep == 22:
            self.Error_Array.append(self.move_robot("get_hose_pose"))
            time.sleep(3)
            self.Error_Array.append(self.move_robot("start_pose"))

        #check the error_array for errors
        for error_value in self.Error_Array:
            if error_value == None:
                Success= True

            else: # publich auf Error topic und sagt welche problem
                #self.get_logger().info(f"Received message from topic error Mode: {self.Error_Array}")
                Success = False
                self.get_logger().error('ERROR: Failure in processstep_callback!')
                self.Error_Array.append(10) # There is an error in  processstep callback add 10

                #self.get_logger().info(f"Received message from topic error Mode: {self.Error_Array}")

                self.get_logger().info(f"Received message from topic error Mode: {self.error_name[error_value]}")
                self.Error_Array = [] # empty the Error_Array

        if Success == True:
            # # Publish ProcessStepSuccess
            # msgsuccess = ProcessStep()
            # msgsuccess.origin = "SchedulerNode"
            # msgsuccess.processstep = self.processstep
            # self.processstepsuccess_publisher_.publish(msgsuccess)
            self.get_logger().info("Finished Processstep and publish it")
            self.process_succeed=True
            self.running_step=False
        else:
            self.get_logger().info("NOT SUCCESS PROCESSSTEP,WAIT")



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

            self.process_succeed = False
            self.running_step = False
            self.processstep = self.processstep + 1

            # # neuen Prozessschritt publishen
            # msg_processtep = ProcessStep()
            # msg_processtep.origin = "SchedulerNode"
            # msg_processtep.processstep = self.processstep
            # self.processstep_publisher_.publish(msg_processtep)
            self.get_logger().info("ALL NODES FINISHED")
        else:
            self.get_logger().info(f"ALL NODES NOT FINISHED:{self.processstep}{self.processstepsuccess_gui}{self.processstepsuccess_rpi}{self.processstepsuccess_scheduler}")