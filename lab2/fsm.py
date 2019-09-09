import sys
import cozmo
import datetime
import time


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
                robot.say_text("drone_test").wait_for_completed()
                timestamp = datetime.datetime.now().strftime("%dT%H%M%S%f")
                new_image.save("./imgs/" + str("drone_test") + "_" + timestamp + ".bmp")
                time.sleep(3)


        if self.observed_symbol == STATES['drone']:
            self.next_state()

        """
        Go to next states
        """

    def next_state(self):
        pass


class Drone(State):
    def __init__(self):
        pass

    def run(self):
        pass

    def next_state(self):
        pass


if __name__ == "__main__":
    state = Idle()
    cozmo.setup_basic_logging()
    try:
        cozmo.connect(state.run)
    except cozmo.ConnectionError as e:
        sys.exit("A connection error occurred: %s" % e)

