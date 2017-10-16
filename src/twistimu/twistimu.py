from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from math import asin, atan2, copysign
from config import Config
from roll import Roll
from pitch import Pitch

class TwistIMU:

    def __init__(self, config=Config()):

        self.config = config
        self.reconfigure()

        self.prev_left_motor_speed = 0
        self.prev_right_motor_speed = 0

    def computeTwist(self, data, elapsed_time):
        """
          Handle input values from IMU
        """

        roll, pitch, yaw = self.computeEulerAngles(data.orientation)
        avg_speed = self.computeAverageSpeedFromRoll(roll)
        speed_offset = self.computeSpeedOffset(avg_speed, pitch, yaw)

        left_motor_speed = self.computeLeftSpeed(
            avg_speed, speed_offset, elapsed_time)
        right_motor_speed = self.computeRightSpeed(
            avg_speed, speed_offset, elapsed_time)

        msg = self.createTwistMessage(left_motor_speed, right_motor_speed)

        self.prev_left_motor_speed = left_motor_speed
        self.prev_right_motor_speed = right_motor_speed

        return msg

    def computeEulerAngles(self, orientation):

        q0 = orientation.x
        q1 = orientation.y
        q2 = orientation.z
        q3 = orientation.w

        roll = Roll(-asin(2 * (q0 * q2 - q3 * q1)) - self.config.roll_neutral)
        pitch = Pitch(atan2(2 * (q0 * q3 + q1 * q2), -1 + 2 * (q2 * q2 + q3 * q3)))
        yaw = atan2(2 * (q0 * q1 + q2 * q3), 1 - 2 * (q1 * q1 + q2 * q2))

        return roll, pitch, yaw

    def createTwistMessage(self, left_motor_speed, right_motor_speed):

        msg = Twist()
        msg.linear.x = (left_motor_speed + right_motor_speed) / 2.0
        msg.angular.z = (left_motor_speed - right_motor_speed) / float(self.config.track)
        return msg

    def computeSpeedOffset(self, avg_speed, pitch, yaw=0.0):

        speed_offset = abs(self.config.speed_neutral + (avg_speed /
                           (self.config.speed_max - self.config.speed_neutral))) * \
            self.computeSpeedOffsetFromPitch(pitch)

        if avg_speed == 0.0 and not pitch.isInDeadZone(self.config):
            speed_offset = abs(self.config.speed_neutral + (2 * self.config.roll_dead_zone /
                               (self.config.speed_max - self.config.speed_neutral))) * \
                self.computeSpeedOffsetFromPitch(pitch)

        return speed_offset

    def computeLeftSpeed(self, avg_speed, speed_offset, elapsed_time):

        left_speed = avg_speed - copysign(speed_offset, (avg_speed - self.config.speed_neutral) * speed_offset)

        left_speed = self.clampSpeed(left_speed)
        acceleration = abs(self.prev_left_motor_speed - left_speed) / elapsed_time

        if acceleration > self.config.max_acceleration:
            left_speed = self.prev_left_motor_speed - \
              copysign(self.config.max_acceleration * elapsed_time, self.prev_left_motor_speed - left_speed)

        return left_speed

    def computeRightSpeed(self, avg_speed, speed_offset, elapsed_time):

        right_speed = avg_speed + copysign(speed_offset, (avg_speed - self.config.speed_neutral) * speed_offset)

        right_speed = self.clampSpeed(right_speed)
        acceleration = abs(self.prev_right_motor_speed - right_speed) / elapsed_time

        if acceleration > self.config.max_acceleration:
            right_speed = self.prev_right_motor_speed - \
              copysign(self.config.max_acceleration * elapsed_time, self.prev_right_motor_speed - right_speed)

        return right_speed

    def computeAverageSpeedFromRoll(self, roll):

        if roll.isInDeadZone(self.config):
            avg_speed = self.config.speed_neutral

        elif roll.isAboveMax(self.config):
            avg_speed = self.config.speed_max

        elif roll.isUnderMin(self.config):
            avg_speed = self.config.speed_min

        elif roll.isForward(self.config):
            avg_speed = self.forward_avg_speed_coeff[1] * roll.value + self.forward_avg_speed_coeff[0]

        elif roll.isBackward(self.config):
            avg_speed = self.backward_avg_speed_coeff[1] * roll.value + self.backward_avg_speed_coeff[0]

        return avg_speed

    def computeSpeedOffsetFromPitch(self, pitch):

        if pitch.isAboveMax(self.config):
            speed_offset = self.config.speed_max

        elif pitch.isUnderMin(self.config):
            speed_offset = self.config.speed_min

        elif pitch.isLeft(self.config):
            speed_offset = self.right_speed_offset_coeff[1] * pitch.value + self.right_speed_offset_coeff[0]

        elif pitch.isRight(self.config):
            speed_offset = self.left_speed_offset_coeff[1] * pitch.value + self.left_speed_offset_coeff[0]

        else:
            speed_offset = 0

        return speed_offset

    def clampSpeed(self, speed):

        clamped_speed = min(speed, self.config.speed_max)
        speed = max(clamped_speed, self.config.speed_min)

        return speed

    def configCallback(self, dyn_config, level):
        """
          Handle dynamic reconfigure update
        """

        self.config.pitch_min = dyn_config["pitch_min"]
        self.config.pitch_neutral = dyn_config["pitch_neutral"]
        self.config.pitch_max = dyn_config["pitch_max"]
        self.config.roll_min = dyn_config["roll_min"]
        self.config.roll_neutral = dyn_config["roll_neutral"]
        self.config.roll_max = dyn_config["roll_max"]
        self.config.speed_min = dyn_config["speed_min"]
        self.config.speed_neutral = dyn_config["speed_neutral"]
        self.config.speed_max = dyn_config["speed_max"]
        self.config.roll_dead_zone = dyn_config["roll_dead_zone"]
        self.config.pitch_dead_zone = dyn_config["pitch_dead_zone"]
        self.config.max_acceleration = dyn_config["max_acceleration"]
        self.config.track = dyn_config["track"]

        self.reconfigure()

        return dyn_config

    def reconfigure(self):
        """
          Update values used by algorithm according to current configuration values
        """

        self.forward_avg_speed_coeff = [0, 0]
        self.forward_avg_speed_coeff[1] = (self.config.speed_max - self.config.speed_neutral) / \
            (self.config.roll_max - self.config.roll_neutral + self.config.roll_dead_zone)
        self.forward_avg_speed_coeff[0] = self.config.speed_neutral - self.forward_avg_speed_coeff[1] * \
            (self.config.roll_neutral + self.config.roll_dead_zone)

        self.backward_avg_speed_coeff = [0, 0]
        self.backward_avg_speed_coeff[1] = (self.config.speed_neutral - self.config.speed_min) / \
            (self.config.roll_neutral - self.config.roll_dead_zone - self.config.roll_min)
        self.backward_avg_speed_coeff[0] = self.config.speed_neutral - self.backward_avg_speed_coeff[1] * \
            (self.config.roll_neutral - self.config.roll_dead_zone)

        self.left_speed_offset_coeff = [0, 0]
        self.left_speed_offset_coeff[1] = self.config.speed_max / (self.config.pitch_neutral - self.config.pitch_min)
        self.left_speed_offset_coeff[0] = -self.left_speed_offset_coeff[1] * self.config.pitch_neutral

        self.right_speed_offset_coeff = [0, 0]
        self.right_speed_offset_coeff[1] = self.config.speed_max / (self.config.pitch_max - self.config.pitch_neutral)
        self.right_speed_offset_coeff[0] = -self.right_speed_offset_coeff[1] * self.config.pitch_neutral