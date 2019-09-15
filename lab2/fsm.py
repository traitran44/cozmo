# Cozmo password: M02HTALN900F
import sys
import cozmo
import datetime
import time
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
                classifier = load('clf.joblib') #needs file that is too big for github
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
        pass

    def object_handler(self, args):
        print(args)

    def run(self, sdk_conn):
        """
        TODO HERE
        """
        robot = sdk_conn.wait_for_robot()
        robot.camera.image_stream_enabled = True
        robot.camera.color_image_enabled = False
        robot.camera.enable_auto_exposure()
        robot.set_head_angle(cozmo.util.degrees(0)).wait_for_completed()

        robot.say_text("drone state").wait_for_completed()

        robot.add_event_handler(cozmo.objects.EvtObjectAppeared, self.object_handler)

        return self.next_state()

    def next_state(self):
        return Idle()


class FSM():
    def __init__(self, start_state: State):
        self.state = start_state

    def run(self, sdk_conn):
        self.sdk_conn = sdk_conn
        while not self.state is None:
            self.state = self.state.run(sdk_conn=self.sdk_conn)

if __name__ == "__main__":
    start_state = Idle()
    fsm = FSM(start_state=start_state)
    cozmo.setup_basic_logging()
    try:
        cozmo.connect(fsm.run)
    except cozmo.ConnectionError as e:
        sys.exit("A connection error occurred: %s" % e)
