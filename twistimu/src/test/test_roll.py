from nose.tools import assert_equal
from nose.tools import assert_almost_equal
from nose.tools import assert_true
from nose.tools import assert_false
from twistimu.twistimu import Config, Roll

def test_roll_isInDeadZone():

    config = Config()
    config.roll_neutral = 0.5
    config.roll_dead_zone = 0.1

    assert_true(Roll(0.5).isInDeadZone(config))
    assert_true(Roll(0.6 - 1e-5).isInDeadZone(config))
    assert_true(Roll(0.45).isInDeadZone(config))

    assert_false(Roll(0.4).isInDeadZone(config))
    assert_false(Roll(0.7).isInDeadZone(config))
    assert_false(Roll(-1.2).isInDeadZone(config))

def test_roll_isAboveMax():

    config = Config()
    config.roll_max = 1

    assert_true(Roll(1.5).isAboveMax(config))
    assert_true(Roll(1.0 + 1e-5).isAboveMax(config))

    assert_false(Roll(1.0).isAboveMax(config))
    assert_false(Roll(-2).isAboveMax(config))

def test_roll_isUnderMin():

    config = Config()
    config.roll_min = 1

    assert_true(Roll(-2).isUnderMin(config))
    assert_true(Roll(1.0 - 1e-5).isUnderMin(config))

    assert_false(Roll(1.5).isUnderMin(config))
    assert_false(Roll(1.0).isUnderMin(config))

def test_roll_isForward():

    config = Config()
    config.roll_neutral = 1

    assert_true(Roll(1.5).isForward(config))
    assert_true(Roll(1.0).isForward(config))

    assert_false(Roll(1.0 - 1e-5).isForward(config))
    assert_false(Roll(-2).isForward(config))

def test_roll_isBackward():

    config = Config()
    config.roll_neutral = 1

    assert_true(Roll(1.0 - 1e-5).isBackward(config))
    assert_true(Roll(-2).isBackward(config))

    assert_false(Roll(1.5).isBackward(config))
    assert_false(Roll(1.0).isBackward(config))