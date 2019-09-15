# Cozmo password: M02HTALN900F
import sys
import cozmo
import datetime
import time
import math
from joblib import load
from imgclassification import ImageClassifier

STATES = {
    'idle': 0,
    'drone': 1,
    'order': 2,
    'inspection': 3
}


class State:
    def __init__(self):
        pass

    def run(self, sdk_conn):
        pass

    def next_state(self):
        pass

    def goto(self):
        pass


class Order(State):
    def __init__(self):
        self.observed_symbol = None
        self.robot = None
        print("Order State")

    def run(self, sdk_conn):
        self.robot = sdk_conn.wait_for_robot()
        self.robot.camera.image_stream_enabled = True
        self.robot.camera.color_image_enabled = False
        self.robot.camera.enable_auto_exposure()
        self.robot.set_head_angle(cozmo.util.degrees(0)).wait_for_completed()

        distance = 250
        drive_duration = 2

        self.robot.drive_straight(cozmo.util.distance_mm(distance), cozmo.util.speed_mmps(distance / drive_duration),
                                  should_play_anim=True,
                                  in_parallel=False,
                                  num_retries=0).wait_for_completed()
        self.robot.drive_wheels(l_wheel_speed=12 * math.pi, r_wheel_speed=50 * math.pi, duration=5)

        while True:
            pass

    def next_state(self):
        return None


class Inspection(State):
    def __init__(self):
        self.observed_symbol = None
        self.robot = None
        print("Order State")

    def run(self, sdk_conn):
        self.robot = sdk_conn.wait_for_robot()
        self.robot.camera.image_stream_enabled = True
        self.robot.camera.color_image_enabled = False
        self.robot.camera.enable_auto_exposure()
        self.robot.set_head_angle(cozmo.util.degrees(0)).wait_for_completed()

        distance = 150
        drive_duration = 2

        self.robot.set_lift_height(height=0, in_parallel=True, num_retries=0)
        self.robot.move_lift(speed=-math.pi)
        drive_action = self.robot.drive_straight(cozmo.util.distance_mm(distance),
                                                 cozmo.util.speed_mmps(distance / drive_duration),
                                                 should_play_anim=True,
                                                 in_parallel=True,
                                                 num_retries=0).wait_for_completed()

        lift_height = True
        distance = 200
        drive_duration = 1
        angle = 90
        rotate_duration = 1
        for i in range(4):
            height = 1.0 if lift_height else 0.0
            lift_speed = math.pi / 15 if lift_height else -math.pi / 15
            print("Height: ", height)
            self.robot.set_lift_height(height=height, in_parallel=True, num_retries=0)
            self.robot.move_lift(speed=lift_speed)
            drive_action = self.robot.drive_straight(cozmo.util.distance_mm(distance),
                                                     cozmo.util.speed_mmps(distance / drive_duration),
                                                     should_play_anim=True,
                                                     in_parallel=True,
                                                     num_retries=0).wait_for_completed()
            turn_action = self.robot.turn_in_place(cozmo.util.degrees(angle), in_parallel=True,
                                                   num_retries=0, speed=cozmo.util.degrees(angle / rotate_duration),
                                                   accel=None, angle_tolerance=None,
                                                   is_absolute=False).wait_for_completed()
            lift_height = not lift_height

        self.robot.backup_onto_charger(max_drive_time=10)

    def next_state(self):
        return None


class Idle(State):
    def __init__(self):
        self.observed_symbol = None
        self.label = None
        print("idle_state")

    def run(self, sdk_conn):
        robot = sdk_conn.wait_for_robot()
        robot.camera.image_stream_enabled = True
        robot.camera.color_image_enabled = False
        robot.camera.enable_auto_exposure()
        robot.set_head_angle(cozmo.util.degrees(0)).wait_for_completed()

        if self.observed_symbol is None:
            while True:
                time.sleep(0.5)
                # Keep observing image
                latest_image = robot.world.latest_image
                new_image = latest_image.raw_image
                classifier = load('clf.joblib')  # needs file that is too big for github
                img_features = ImageClassifier.extract_image_features(classifier, [new_image])
                self.label = classifier.predict(img_features)[0]
                robot.say_text(self.label).wait_for_completed()
                timestamp = datetime.datetime.now().strftime("%dT%H%M%S%f")
                new_image.save("./imgs/" + str(self.label) + "_" + timestamp + ".bmp")

                next = self.next_state()

                if not next is None:
                    return next

        """
        Go to next states
        """

    def next_state(self):
        if self.label == "drone":
            return Drone()
        return None


class Drone(State):
    def __init__(self):
        self.robot = None
        self.picking_up = False

    def object_handler(self, *args, **kwargs):
        # for arg in args:
        #     print(arg)
        # for key, val in kwargs.items():
        #     print(key, ": ", val)
        # self.robot.stop_all_motors().wait_for_completed()
        if not self.picking_up:
            print(kwargs['obj'])
            self.picking_up = True
            self.robot.dock_with_cube(kwargs['obj'],
                                      alignment_type=cozmo.robot_alignment.RobotAlignmentTypes.LiftFinger,
                                      in_parallel=False,
                                      num_retries=3).wait_for_completed()
            self.robot.set_lift_height(height=1, in_parallel=True, num_retries=1).wait_for_completed()
            self.robot.move_lift(math.pi / 4)

    def run(self, sdk_conn):
        """
        TODO HERE
        """
        self.robot = sdk_conn.wait_for_robot()
        self.robot.camera.image_stream_enabled = True
        self.robot.camera.color_image_enabled = False
        self.robot.camera.enable_auto_exposure()
        self.robot.set_head_angle(cozmo.util.degrees(0)).wait_for_completed()

        self.robot.say_text("drone state").wait_for_completed()

        self.robot.set_lift_height(height=0, in_parallel=False, num_retries=0).wait_for_completed()
        self.robot.move_lift(speed=-math.pi)

        distance = 200
        drive_duration = 2
        self.robot.drive_straight(cozmo.util.distance_mm(distance), cozmo.util.speed_mmps(distance / drive_duration),
                                  should_play_anim=True,
                                  in_parallel=False,
                                  num_retries=0).wait_for_completed()

        angle = 90
        rotate_duration = 1
        self.robot.turn_in_place(cozmo.util.degrees(angle), in_parallel=False,
                                 num_retries=0, speed=cozmo.util.degrees(angle / rotate_duration),
                                 accel=None, angle_tolerance=None, is_absolute=False).wait_for_completed()

        self.robot.add_event_handler(cozmo.objects.EvtObjectObserved, self.object_handler)

        while True:
            pass

        # return self.next_state()

    def next_state(self):
        return Idle()


class FSM():
    def __init__(self, start_state: State):
        self.state = start_state

    def run(self, sdk_conn):
        self.sdk_conn = sdk_conn
        # while not self.state is None:
        self.state = self.state.run(sdk_conn=self.sdk_conn)


if __name__ == "__main__":
    start_state = Drone()
    fsm = FSM(start_state=start_state)
    cozmo.setup_basic_logging()
    try:
        cozmo.connect(fsm.run)
    except cozmo.ConnectionError as e:
        sys.exit("A connection error occurred: %s" % e)
