import time


class HLC:
    ...


class ProcessStep:
    ...


class Node:
    ...


class SchedulerNode(Node):
    def __init__(self) -> None:
        ...
        self.succeeded = True  # NOTE: initially set to True for the 1. process step
        ...

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

        # Endzustände
        while requested_processstep not in {3, 5, 6, 20, 21, 22, 23}:
            self.processstep = requested_processstep
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info("Goal successfully canceled from client.")
                return get_result(success=False)

            if self.succeeded:  # if the previous process step was successful, execute the next one
                publish_processstep()
                self.succeeded = False
            # Fetch the result from the result from scheduler node: Error/Success
            # TODO: inside processstep_callback(), set a global var to sign `self.succeed` if it is successful or not
            time.sleep(1)
            self.get_logger().info("Waiting for the result of the process step...")
            time.sleep(1)
            if self.succeeded:  # basically when gui, rpi, scheduler nodes are all ready...
                publish_success()  # it must be done here in action server, not in `processstep_callback()`
                # must be done here in action server, not in `processstep_callback()`
                requested_processstep += 1

        goal_handle.succeed()
        self.get_logger().info("Goal succeeded")

        return get_result(success=True)
