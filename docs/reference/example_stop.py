#!/usr/bin/env python3

from datetime import datetime

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from dsr_msgs2.srv import (
    MoveJoint,
    MoveLine,
    MovePause,
    MoveResume,
    MoveStop,
    CheckMotion,
    GetRobotState,
)


ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"

_srv_name_prefix = "dsr_controller2"

MOVE_LINE_SERVICE       = f"/{ROBOT_ID}/{_srv_name_prefix}/motion/move_line"
MOVE_JOINT_SERVICE      = f"/{ROBOT_ID}/{_srv_name_prefix}/motion/move_joint"
MOVE_PAUSE_SERVICE      = f"/{ROBOT_ID}/{_srv_name_prefix}/motion/move_pause"
MOVE_RESUME_SERVICE     = f"/{ROBOT_ID}/{_srv_name_prefix}/motion/move_resume"
MOVE_STOP_SERVICE       = f"/{ROBOT_ID}/{_srv_name_prefix}/motion/move_stop"
CHECK_MOTION_SERVICE    = f"/{ROBOT_ID}/{_srv_name_prefix}/motion/check_motion"
GET_ROBOT_STATE_SERVICE = f"/{ROBOT_ID}/{_srv_name_prefix}/system/get_robot_state"


DR_STATE_IDLE = 0
DR_STATE_INIT = 1
DR_STATE_BUSY = 2

DR_STATE_NAMES = {
    0: "DR_STATE_IDLE",
    1: "DR_STATE_INIT",
    2: "DR_STATE_BUSY",
}


ROBOT_STATE_INITIALIZING = 0
ROBOT_STATE_STANDBY = 1
ROBOT_STATE_MOVING = 2
ROBOT_STATE_SAFE_OFF = 3
ROBOT_STATE_TEACHING = 4
ROBOT_STATE_SAFE_STOP = 5
ROBOT_STATE_EMERGENCY_STOP = 6
ROBOT_STATE_HOMING = 7
ROBOT_STATE_RECOVERY = 8
ROBOT_STATE_SAFE_STOP2 = 9
ROBOT_STATE_SAFE_OFF2 = 10
ROBOT_STATE_NOT_READY = 15

ROBOT_STATE_NAMES = {
    0: "INITIALIZING",
    1: "STANDBY",
    2: "MOVING",
    3: "SAFE_OFF",
    4: "TEACHING",
    5: "SAFE_STOP",
    6: "EMERGENCY_STOP",
    7: "HOMING",
    8: "RECOVERY",
    9: "SAFE_STOP2",
    10: "SAFE_OFF2",
    15: "NOT_READY",
}


MOVE_LINE_TIME = 10.0

MOVE_LINE_START_DELAY = 0.5
MOTION_CHECK_INTERVAL = 0.2
MOVE_LINE_BUSY_TIMEOUT = 3.0

PAUSE_AFTER_BUSY_SEC = 3.0
PAUSE_WAIT_SEC = 1.0

RESUME_BUSY_TIMEOUT = 3.0
RESUME_BEFORE_STOP_SEC = 1.0

STOP_CHECK_DELAY_SEC = 3.0


class MovePauseResumeTest(Node):
    def __init__(self):

        super().__init__("move_pause_resume_test")

        self.move_line_client    = self.create_client(MoveLine, MOVE_LINE_SERVICE)
        self.move_joint_client   = self.create_client(MoveJoint, MOVE_JOINT_SERVICE)
        self.pause_client        = self.create_client(MovePause, MOVE_PAUSE_SERVICE)
        self.resume_client       = self.create_client(MoveResume, MOVE_RESUME_SERVICE)
        self.stop_client         = self.create_client(MoveStop, MOVE_STOP_SERVICE)
        self.check_motion_client = self.create_client(CheckMotion, CHECK_MOTION_SERVICE)
        self.robot_state_client  = self.create_client(
            GetRobotState, GET_ROBOT_STATE_SERVICE
        )

        self.test_step = 0

        self.test_running = False
        self.test_finished = False

        self.command_running = False
        self.check_motion_running = False

        self.step_start_time = None

        self.move_line_start_time = None
        self.resume_start_time = None

        self.test_timer = self.create_timer(0.2, self.test_loop)

        self.log("==========================================")
        self.log(" Doosan Move Pause/Resume/Stop Test")
        self.log(" CheckMotion based motion synchronization")
        self.log("==========================================")

        self.wait_for_services()

    def timestamp(self):

        return datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]

    def log(self, message):

        print(f"[{self.timestamp()}] [MOVE-TEST] {message}", flush=True)

    def elapsed(self):

        if self.step_start_time is None:
            return 0.0

        duration = self.get_clock().now() - self.step_start_time

        return duration.nanoseconds / 1e9

    def wait_for_services(self):

        services = [
            ("MoveLine", self.move_line_client),
            ("MoveJoint", self.move_joint_client),
            ("MovePause", self.pause_client),
            ("MoveResume", self.resume_client),
            ("MoveStop", self.stop_client),
            ("CheckMotion", self.check_motion_client),
            ("GetRobotState", self.robot_state_client),
        ]

        for name, client in services:
            self.log(f"Waiting for {name} service...")

            if not client.wait_for_service(timeout_sec=10.0):
                raise RuntimeError(f"{name} service is not available")

            self.log(f"{name} service connected")

    def start_test(self):

        if self.test_running:
            return

        self.log("==========================================")
        self.log("TEST START")
        self.log("==========================================")

        self.test_running = True
        self.test_finished = False
        self.test_step = 0

        self.command_running = False
        self.check_motion_running = False

        self.step_start_time = self.get_clock().now()

    def get_robot_state(self, callback):

        request = GetRobotState.Request()

        future = self.robot_state_client.call_async(request)

        future.add_done_callback(lambda future: self.robot_state_done(future, callback))

    def robot_state_done(self, future, callback):

        try:
            result = future.result()

            if result is None:
                self.log("GetRobotState ERROR: no response")

                callback(None)
                return

            if not result.success:
                self.log("GetRobotState FAILED")

                callback(None)
                return

            state = int(result.robot_state)

            state_name = ROBOT_STATE_NAMES.get(state, f"UNKNOWN({state})")

            self.log(f"ROBOT STATE = {state} ({state_name})")

            callback(state)

        except Exception as e:
            self.log(f"GetRobotState ERROR: {e}")

            callback(None)

    def check_motion(self, callback, description=""):

        if self.check_motion_running:
            return

        self.check_motion_running = True

        request = CheckMotion.Request()

        text = ""

        if description:
            text = f" - {description}"

        self.log(f"CheckMotion REQUEST{text}")

        future = self.check_motion_client.call_async(request)

        future.add_done_callback(
            lambda future: self.check_motion_done(future, callback, description)
        )

    def check_motion_done(self, future, callback, description):

        self.check_motion_running = False

        try:
            result = future.result()

            if result is None:
                self.log("CheckMotion ERROR: no response")

                callback(None)
                return

            if not result.success:
                self.log("CheckMotion FAILED")

                callback(None)
                return

            status = int(result.status)

            name = DR_STATE_NAMES.get(status, f"UNKNOWN({status})")

            self.log(
                f"CheckMotion RESPONSE "
                f"status={status} "
                f"({name}), "
                f"success={result.success}"
            )

            callback(status)

        except Exception as e:
            self.log(f"CheckMotion ERROR: {e}")

            callback(None)

    def wait_for_idle(self, callback, description=""):

        if self.test_finished:
            return

        if self.check_motion_running:
            return

        self.check_motion(
            lambda status: self.idle_callback(status, callback, description),
            description,
        )

    def idle_callback(self, status, callback, description):

        if self.test_finished:
            return

        if status is None:
            self.handle_motion_error(f"CheckMotion failed: {description}")

            return

        if status == DR_STATE_IDLE:
            self.log(f"CheckMotion = IDLE. Next operation allowed. {description}")

            callback()
            return

        if status == DR_STATE_INIT:
            self.log("CheckMotion = INIT. Waiting...")

            return

        if status == DR_STATE_BUSY:
            self.log("CheckMotion = BUSY. Waiting...")

            return

        self.handle_motion_error(f"Unknown CheckMotion status={status}")

    def start_home(self):

        self.log("------------------------------------------")
        self.log("STEP 0: Robot State Check")
        self.log("------------------------------------------")

        self.command_running = True

        self.get_robot_state(self.home_state_cb)

    def home_state_cb(self, state):

        if state is None:
            self.command_running = False

            self.handle_motion_error("Home: robot state unavailable")

            return

        name = ROBOT_STATE_NAMES.get(state, f"UNKNOWN({state})")

        self.log(f"Home pre-check: state={state} ({name})")

        if state != ROBOT_STATE_STANDBY:
            self.command_running = False

            self.handle_motion_error(f"Cannot start Home. Current state={name}")

            return

        self.log("Robot state is STANDBY.")

        self.log("Checking CheckMotion before Home...")

        self.wait_for_idle(self.execute_home, "Before MoveJoint Home")

    def execute_home(self):

        request = MoveJoint.Request()

        request.pos = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]

        request.vel = 60.0
        request.acc = 60.0
        request.time = 0.0
        request.radius = 0.0
        request.mode = 0
        request.blend_type = 0
        request.sync_type = 0

        self.log(f"MoveJoint START target={request.pos}")

        future = self.move_joint_client.call_async(request)

        future.add_done_callback(self.move_joint_done)

    def move_joint_done(self, future):

        try:
            result = future.result()

            if result is None:
                self.command_running = False

                self.handle_motion_error("MoveJoint: no response")

                return

            self.log(f"MoveJoint RESPONSE success={result.success}")

            if not result.success:
                self.command_running = False

                self.handle_motion_error("MoveJoint failed")

                return

            self.log("MoveJoint command accepted.")

            self.command_running = False

            self.test_step = 1

            self.step_start_time = self.get_clock().now()

        except Exception as e:
            self.command_running = False

            self.handle_motion_error(f"MoveJoint exception: {e}")

    def start_move_line(self):

        self.log("------------------------------------------")
        self.log("STEP 1: MoveLine")
        self.log("------------------------------------------")

        self.command_running = True

        self.log("Checking CheckMotion before MoveLine...")

        self.wait_for_idle(self.execute_move_line, "Before MoveLine")

    def execute_move_line(self):

        request = MoveLine.Request()

        request.pos        = [600.0, 0.0, 400.0, 0.0, 90.0, 0.0]
        request.vel        = [100.0, 30.0]
        request.acc        = [100.0, 30.0]
        request.time       = MOVE_LINE_TIME
        request.radius     = 0.0
        request.ref        = 0
        request.mode       = 0
        request.blend_type = 0
        request.sync_type  = 1

        self.log(f"MoveLine START target={request.pos}")
        self.log(f"MoveLine time={MOVE_LINE_TIME:.1f}s")
        self.log("MoveLine is running asynchronously.")

        future = self.move_line_client.call_async(request)
        future.add_done_callback(self.move_line_done)

    def move_line_done(self, future):

        try:
            result = future.result()

            if result is None:
                self.command_running = False

                self.handle_motion_error("MoveLine: no response")

                return

            self.log(f"MoveLine RESPONSE success={result.success}")

            if not result.success:
                self.command_running = False

                self.handle_motion_error("MoveLine failed")

                return

            self.log("MoveLine command accepted.")

            self.command_running = False

            self.move_line_start_time = self.get_clock().now()

            self.step_start_time = self.get_clock().now()

            self.test_step = 2

        except Exception as e:
            self.command_running = False

            self.handle_motion_error(f"MoveLine exception: {e}")

    def start_pause_check(self):

        if self.test_finished:
            return

        if self.elapsed() < MOVE_LINE_START_DELAY:
            return

        self.log("------------------------------------------")
        self.log("STEP 2: Pause")
        self.log("------------------------------------------")

        self.log("Checking CheckMotion for active MoveLine...")

        self.test_step = 21

        self.step_start_time = self.get_clock().now()

    def process_move_line_busy_check(self):

        if self.test_finished:
            return

        if self.check_motion_running:
            return

        if self.elapsed() < MOTION_CHECK_INTERVAL:
            return

        if self.move_line_start_time is None:
            self.handle_motion_error("MoveLine start time unavailable")

            return

        elapsed = (self.get_clock().now() - self.move_line_start_time).nanoseconds / 1e9

        if elapsed > MOVE_LINE_BUSY_TIMEOUT:
            self.handle_motion_error(
                f"MoveLine did not become BUSY within {MOVE_LINE_BUSY_TIMEOUT:.1f}s"
            )

            return

        self.step_start_time = self.get_clock().now()

        self.check_motion(self.move_line_busy_cb, "Before MovePause")

    def move_line_busy_cb(self, status):

        if self.test_finished:
            return

        if status is None:
            self.handle_motion_error(
                "CheckMotion failed while waiting for MoveLine BUSY"
            )

            return

        if status == DR_STATE_BUSY:
            self.log("CheckMotion = BUSY.")

            self.log("Active MoveLine confirmed.")

            self.test_step = 22

            self.step_start_time = self.get_clock().now()

            return

        if status == DR_STATE_INIT:
            self.log("CheckMotion = INIT. Waiting for BUSY...")

            return

        if status == DR_STATE_IDLE:
            self.log("CheckMotion = IDLE. MoveLine not active yet.")

            return

        self.handle_motion_error(f"Unknown CheckMotion status={status}")

    def process_pause_delay(self):

        if self.test_finished:
            return

        if self.elapsed() < PAUSE_AFTER_BUSY_SEC:
            return

        self.command_running = True

        self.log("Checking CheckMotion immediately before MovePause...")

        self.check_motion(self.pause_before_cb, "Immediately Before MovePause")

    def pause_before_cb(self, status):

        if self.test_finished:
            return

        if status is None:
            self.command_running = False

            self.handle_motion_error("Pause pre-check failed")

            return

        if status == DR_STATE_BUSY:
            self.log("CheckMotion = BUSY. Active motion confirmed.")

            self.execute_pause()

            return

        if status == DR_STATE_INIT:
            self.command_running = False

            self.test_step = 21

            self.step_start_time = self.get_clock().now()

            return

        if status == DR_STATE_IDLE:
            self.command_running = False

            self.handle_motion_error("MoveLine became IDLE before Pause")

            return

        self.command_running = False

        self.handle_motion_error(f"Unknown CheckMotion status={status}")

    def execute_pause(self):

        request = MovePause.Request()

        self.log("MovePause REQUEST")

        future = self.pause_client.call_async(request)

        future.add_done_callback(self.move_pause_done)

    def move_pause_done(self, future):

        try:
            result = future.result()

            if result is None:
                self.command_running = False

                self.handle_motion_error("MovePause: no response")

                return

            self.log(f"MovePause RESPONSE success={result.success}")

            if not result.success:
                self.command_running = False

                self.handle_motion_error("MovePause failed")

                return

            self.log("MovePause command accepted.")
            self.log(f"Pause 유지 시간 {PAUSE_WAIT_SEC:.1f}s")

            self.command_running = False

            self.test_step = 3

            self.step_start_time = self.get_clock().now()

        except Exception as e:
            self.command_running = False

            self.handle_motion_error(f"MovePause exception: {e}")

    def start_resume(self):

        self.log("------------------------------------------")
        self.log("STEP 3: Resume")
        self.log("------------------------------------------")

        self.step_start_time = self.get_clock().now()

        self.test_step = 30

    def process_resume_wait(self):

        if self.test_finished:
            return

        if self.elapsed() < PAUSE_WAIT_SEC:
            return

        self.command_running = True

        self.log("Checking robot state before Resume...")

        self.get_robot_state(self.resume_state_cb)

    def resume_state_cb(self, state):

        if state is None:
            self.command_running = False

            self.handle_motion_error("Resume: robot state unavailable")

            return

        name = ROBOT_STATE_NAMES.get(state, f"UNKNOWN({state})")

        self.log(f"Resume pre-check: state={state} ({name})")

        blocked = {
            ROBOT_STATE_SAFE_OFF,
            ROBOT_STATE_SAFE_STOP,
            ROBOT_STATE_EMERGENCY_STOP,
            ROBOT_STATE_SAFE_STOP2,
            ROBOT_STATE_SAFE_OFF2,
        }

        if state in blocked:
            self.command_running = False

            self.handle_motion_error(f"Resume blocked: {name}")

            return

        request = MoveResume.Request()

        self.log("MoveResume REQUEST")

        future = self.resume_client.call_async(request)

        future.add_done_callback(self.move_resume_done)

    def move_resume_done(self, future):

        try:
            result = future.result()

            if result is None:
                self.command_running = False

                self.handle_motion_error("MoveResume: no response")

                return

            self.log(f"MoveResume RESPONSE success={result.success}")

            if not result.success:
                self.command_running = False

                self.log("MoveResume FAILED")

                self.get_robot_state(self.resume_failed_state_cb)

                return

            self.log("MoveResume command accepted.")

            self.command_running = False

            self.resume_start_time = self.get_clock().now()

            self.step_start_time = self.get_clock().now()

            self.test_step = 4

        except Exception as e:
            self.command_running = False

            self.handle_motion_error(f"MoveResume exception: {e}")

    def resume_failed_state_cb(self, state):

        self.command_running = False

        if state is None:
            self.handle_motion_error("MoveResume failed")

            return

        name = ROBOT_STATE_NAMES.get(state, f"UNKNOWN({state})")

        self.log("==========================================")
        self.log(f"Resume failed robot state = {state} ({name})")
        self.log("==========================================")

        self.handle_motion_error(f"MoveResume failed. Robot state={name}")

    def start_resume_busy_check(self):

        if self.test_finished:
            return

        if self.check_motion_running:
            return

        if self.resume_start_time is None:
            self.handle_motion_error("Resume start time unavailable")

            return

        elapsed = (self.get_clock().now() - self.resume_start_time).nanoseconds / 1e9

        if elapsed > RESUME_BUSY_TIMEOUT:
            self.handle_motion_error(
                f"Resume did not become BUSY within {RESUME_BUSY_TIMEOUT:.1f}s"
            )

            return

        if self.elapsed() < MOTION_CHECK_INTERVAL:
            return

        self.check_motion(self.resume_busy_cb, "After MoveResume")

    def resume_busy_cb(self, status):

        if self.test_finished:
            return

        if status is None:
            self.handle_motion_error("CheckMotion failed after Resume")

            return

        if status == DR_STATE_BUSY:
            self.log("CheckMotion = BUSY.")

            self.log("Resume motion confirmed.")

            self.test_step = 40

            self.step_start_time = self.get_clock().now()

            return

        if status == DR_STATE_INIT:
            self.log("CheckMotion = INIT. Waiting for BUSY...")
            return

        if status == DR_STATE_IDLE:
            self.log("CheckMotion = IDLE. Waiting for Resume motion...")
            return

        self.handle_motion_error(f"Unknown CheckMotion status={status}")

    def process_stop_wait(self):

        if self.test_finished:
            return

        if self.elapsed() < RESUME_BEFORE_STOP_SEC:
            return

        self.command_running = True

        self.log("Checking CheckMotion before Stop...")

        self.check_motion(self.stop_before_cb, "Before MoveStop")

    def stop_before_cb(self, status):

        if self.test_finished:
            return

        if status is None:
            self.command_running = False
            self.handle_motion_error("Stop pre-check failed")
            return

        if status == DR_STATE_BUSY:
            self.log("CheckMotion = BUSY.")
            self.log("Active motion confirmed before Stop.")
            self.execute_stop()
            return

        if status == DR_STATE_INIT:
            self.command_running = False
            self.test_step = 40
            self.step_start_time = self.get_clock().now()
            return

        if status == DR_STATE_IDLE:
            self.log("CheckMotion = IDLE before Stop.")
            self.log("Resume motion already completed.")
            self.command_running = False
            self.test_step = 5
            self.step_start_time = self.get_clock().now()
            return

        self.command_running = False

        self.handle_motion_error(f"Unknown CheckMotion status={status}")

    def execute_stop(self):

        request = MoveStop.Request()

        request.stop_mode = 2

        self.log("MoveStop REQUEST stop_mode=2")

        future = self.stop_client.call_async(request)

        future.add_done_callback(self.move_stop_done)

    def move_stop_done(self, future):

        try:
            result = future.result()

            if result is None:
                self.command_running = False

                self.handle_motion_error("MoveStop: no response")

                return

            self.log(f"MoveStop RESPONSE success={result.success}")

            if not result.success:
                self.command_running = False

                self.handle_motion_error("MoveStop failed")

                return

            self.log("MoveStop command accepted.")

            self.command_running = False

            self.test_step = 5

            self.step_start_time = self.get_clock().now()

        except Exception as e:
            self.command_running = False

            self.handle_motion_error(f"MoveStop exception: {e}")

    def process_finish_check(self):

        if self.test_finished:
            return

        if self.elapsed() < STOP_CHECK_DELAY_SEC:
            return

        if self.check_motion_running:
            return

        self.command_running = True

        self.log("Checking CheckMotion after Stop...")

        self.test_step = 6

        self.check_motion(self.finish_motion_cb, "After MoveStop")

    def finish_motion_cb(self, status):

        if self.test_finished:
            return

        self.command_running = False

        if status is None:
            self.handle_motion_error("CheckMotion after Stop failed")

            return

        if status == DR_STATE_IDLE:
            self.log("CheckMotion = IDLE after Stop.")

            self.finish_test()

            return

        if status == DR_STATE_INIT:
            self.log("CheckMotion = INIT after Stop.")

            self.test_step = 5

            self.step_start_time = self.get_clock().now()

            return

        if status == DR_STATE_BUSY:
            self.log("CheckMotion = BUSY after Stop.")

            self.test_step = 5

            self.step_start_time = self.get_clock().now()

            return

        self.handle_motion_error(f"Unknown CheckMotion status={status}")

    def finish_test(self):

        if self.test_finished:
            return

        self.log("==========================================")
        self.log("TEST END")
        self.log("==========================================")

        self.test_running = False
        self.test_finished = True
        self.command_running = False

        if self.test_timer is not None:
            self.test_timer.cancel()

    def handle_motion_error(self, message):

        self.log("==========================================")
        self.log(f"MOTION ERROR: {message}")
        self.log("==========================================")

        self.test_running = False
        self.test_finished = True

        self.command_running = False
        self.check_motion_running = False

        if self.test_timer is not None:
            self.test_timer.cancel()

    def test_loop(self):

        if self.test_finished:
            return

        if not self.test_running:
            self.start_test()
            self.start_home()

            return

        if self.command_running:
            return

        if self.test_step == 1:
            self.start_move_line()
            return

        if self.test_step == 2:
            self.start_pause_check()
            return

        if self.test_step == 21:
            self.process_move_line_busy_check()
            return

        if self.test_step == 22:
            self.process_pause_delay()
            return

        if self.test_step == 3:
            self.start_resume()
            return

        if self.test_step == 30:
            self.process_resume_wait()
            return

        if self.test_step == 4:
            self.start_resume_busy_check()
            return

        if self.test_step == 40:
            self.process_stop_wait()
            return

        if self.test_step == 5:
            self.process_finish_check()
            return

        if self.test_step == 6:
            return


def main(args=None):

    rclpy.init(args=args)

    node = None
    executor = None

    try:
        node = MovePauseResumeTest()
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(node)
        node.log("Executor started")

        while rclpy.ok() and not node.test_finished:
            executor.spin_once(timeout_sec=0.1)

    except KeyboardInterrupt:
        if node is not None:
            node.log("KeyboardInterrupt")

    except Exception as e:
        if node is not None:
            node.log(f"EXCEPTION: {e}")
        else:
            print(f"EXCEPTION: {e}", flush=True)

    finally:
        if node is not None:
            node.log("Shutting down executor...")

        if executor is not None:
            executor.shutdown()

        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

        print("ROS shutdown complete", flush=True)


if __name__ == "__main__":
    main()
