from nose.tools import assert_equal
from nose.tools import assert_almost_equal
from nose.tools import assert_true
from nose.tools import assert_false

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion

from twistimu import Config
from twistimu import TwistIMU
from twistimu import Pitch
from twistimu import Roll

def test_twistimu_init():

    twistimu = TwistIMU()

def test_twistimu_computeEulerAngles():

    config = Config()
    config.roll_neutral = 1.5

    orientation = Quaternion(0.70149, 0.12821, -0.12343, -0.6901)

    twistimu = TwistIMU(config)
    roll, pitch, yaw = twistimu.computeEulerAngles(orientation)

    assert_almost_equal(roll.value, -1.5037856)
    assert_almost_equal(pitch.value, -1.5878513)
    assert_almost_equal(yaw, 0.35782378)

def assert_twist_almost_equal(msg, result):

    assert_almost_equal(msg.linear.x, result.linear.x)
    assert_almost_equal(msg.linear.y, result.linear.y)
    assert_almost_equal(msg.linear.z, result.linear.z)
    assert_almost_equal(msg.angular.x, result.angular.x)
    assert_almost_equal(msg.angular.y, result.angular.y)
    assert_almost_equal(msg.angular.z, result.angular.z)

def test_twistimu_createTwistMessage():

    config = Config()
    config.track = 1.5
    twistimu = TwistIMU(config)

    result = Twist()
    result.linear.x = 0.6
    result.angular.z = -1.4666666666666668

    msg = twistimu.createTwistMessage(-0.5, 1.7)

    assert_twist_almost_equal(msg, result)

def test_twistimu_computeTwist():

    config = Config()
    config.pitch_min = -2
    config.pitch_neutral = 0
    config.pitch_max = 2
    config.roll_min = -0.8
    config.roll_neutral = 0.0
    config.roll_max = 0.8
    config.speed_min = -2
    config.speed_neutral = 0
    config.speed_max = 2
    config.roll_dead_zone = 0.1
    config.max_acceleration = 10
    twistimu = TwistIMU(config)

    message = Imu()
    message.orientation = Quaternion(0.8410975431166635,
                                     -0.4688013733610667,
                                     -0.23127455271427264,
                                     0.1388966398610968)

    result = Twist()
    result.linear.x = 0.05
    result.angular.z = -0.1

    assert_twist_almost_equal(twistimu.computeTwist(message, 0.01), result)

def test_twistimu_clampSpeed():

    config = Config()
    config.speed_max = 1.2
    config.speed_min = -8
    twistimu = TwistIMU(config)

    assert_almost_equal(twistimu.clampSpeed(1.0), 1.0)
    assert_almost_equal(twistimu.clampSpeed(56), 1.2)
    assert_almost_equal(twistimu.clampSpeed(-8), -8)
    assert_almost_equal(twistimu.clampSpeed(-8 - 1e-5), -8)

def test_twistimu_computeSpeedOffset():

    config = Config()
    config.speed_max = 1.2
    config.speed_neutral = -8
    twistimu = TwistIMU(config)

    assert_almost_equal(twistimu.computeSpeedOffset(0.5, Pitch(-0.24)),
                        -1.1441739130434783)
    assert_almost_equal(twistimu.computeSpeedOffset(0, Pitch(-1.24)),
                        -5.935826086956522)

def test_twistimu_computeLeftSpeed():

    config = Config()
    config.pitch_min = -2
    config.pitch_neutral = 0
    config.pitch_max = 2
    config.roll_min = -0.8
    config.roll_neutral = 0.0
    config.roll_max = 0.8
    config.speed_min = -2
    config.speed_neutral = 0
    config.speed_max = 2
    config.roll_dead_zone = 0.1
    config.max_acceleration = 10
    twistimu = TwistIMU(config)

    assert_almost_equal(twistimu.computeLeftSpeed(0.5, -0.24, 1), 0.74)
    assert_almost_equal(twistimu.computeLeftSpeed(0.5, -0.24, 0.01), 0.1)

def test_twistimu_computeRightSpeed():

    config = Config()
    config.pitch_min = -2
    config.pitch_neutral = 0
    config.pitch_max = 2
    config.roll_min = -0.8
    config.roll_neutral = 0.0
    config.roll_max = 0.8
    config.speed_min = -2
    config.speed_neutral = 0
    config.speed_max = 2
    config.roll_dead_zone = 0.1
    config.max_acceleration = 10
    twistimu = TwistIMU(config)

    assert_almost_equal(twistimu.computeRightSpeed(0.5, -0.24, 1), 0.26)
    assert_almost_equal(twistimu.computeRightSpeed(0.5, -0.24, 0.01), 0.1)

def test_twistimu_computeAverageSpeedFromRoll():

    config = Config()
    config.roll_min = -0.8
    config.roll_neutral = 0.0
    config.roll_max = 0.8
    config.speed_min = -2
    config.speed_neutral = 0
    config.speed_max = 2
    config.roll_dead_zone = 0.1
    twistimu = TwistIMU(config)

    assert_almost_equal(twistimu.computeAverageSpeedFromRoll(Roll(0.23)),
                        0.288888888888889)
    assert_almost_equal(twistimu.computeAverageSpeedFromRoll(Roll(1.24)),
                        2.0)
    assert_almost_equal(twistimu.computeAverageSpeedFromRoll(Roll(-0.02)),
                        0.0)
    assert_almost_equal(twistimu.computeAverageSpeedFromRoll(Roll(-1.02)),
                        -2.0)
    assert_almost_equal(twistimu.computeAverageSpeedFromRoll(Roll(-0.5)),
                        -1.1428571428571428)

def test_twistimu_computeSpeedOffsetFromPitch():

    config = Config()
    config.pitch_min = -2
    config.pitch_neutral = 0
    config.pitch_max = 1.5
    config.speed_min = -2
    config.speed_neutral = 0
    config.speed_max = 2
    twistimu = TwistIMU(config)

    assert_almost_equal(twistimu.computeSpeedOffsetFromPitch(Pitch(0.63)),
                        0.84)
    assert_almost_equal(twistimu.computeSpeedOffsetFromPitch(Pitch(3.24)),
                        2.0)
    assert_almost_equal(twistimu.computeSpeedOffsetFromPitch(Pitch(-0.02)),
                        -0.02)
