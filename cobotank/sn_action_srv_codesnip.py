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
        self.process_succeed = False
        self.running_step = False
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
            msg.processstep = requested_processstep
            self.processstepsuccess_publisher_.publish(msg)

        # Endzustände
        while requested_processstep not in {3, 5, 6, 20, 21, 22, 23}:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info("Goal successfully canceled from client.")
                self.process_succeed = False
                self.running_step = False
                return get_result(success=False)

            if not self.process_succeed and not self.running_step:
                self.get_logger().info(
                    f"Running processstep \`{requested_processstep}\`.")
                publish_processstep()
                self.running_step = True
                self.process_succeed = False

            elif not self.process_succeed and self.running_step:
                self.get_logger().info(
                    f"Loop again till the previous step \`{requested_processstep-1}\` is done.")
                time.sleep(1)
                continue

            elif self.process_succeed and not self.running_step:
                publish_success()
                self.get_logger().info(
                    f"Step \`{requested_processstep-1}\` is done.")
                self.process_succeed = False
                self.running_step = False
                requested_processstep += 1
            else:
                self.get_logger().info(
                    f"Error state at step \`{requested_processstep}\`.")
                return get_result(success=False)

        goal_handle.succeed()
        self.get_logger().info("Goal succeeded")

        return get_result(success=True)
