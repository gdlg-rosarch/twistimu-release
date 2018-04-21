import rospy

class Hourglass:

    def __init__(self):

        self.last_update = None

    def elapsedTime(self):

        if self.last_update is None:
            self.last_update = rospy.get_rostime()

        current_time = rospy.get_rostime()
        elapsed_time = current_time.to_sec() - self.last_update.to_sec()
        self.last_update = current_time

        return elapsed_time