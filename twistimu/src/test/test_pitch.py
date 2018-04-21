from nose.tools import assert_equal
from nose.tools import assert_almost_equal
from nose.tools import assert_true
from nose.tools import assert_false
from twistimu.twistimu import Config, Pitch

def test_pitch_isInDeadZone():

    config = Config()
    config.pitch_neutral = 0.5
    config.pitch_dead_zone = 0.1

    assert_true(Pitch(0.5).isInDeadZone(config))
    assert_true(Pitch(0.6 - 1e-5).isInDeadZone(config))
    assert_true(Pitch(0.45).isInDeadZone(config))

    assert_false(Pitch(0.4).isInDeadZone(config))
    assert_false(Pitch(0.7).isInDeadZone(config))
    assert_false(Pitch(-1.2).isInDeadZone(config))

def test_pitch_isAboveMax():

    config = Config()
    config.pitch_max = 1

    assert_true(Pitch(1.5).isAboveMax(config))
    assert_true(Pitch(1.0 + 1e-5).isAboveMax(config))

    assert_false(Pitch(1.0).isAboveMax(config))
    assert_false(Pitch(-2).isAboveMax(config))

def test_pitch_isUnderMin():

    config = Config()
    config.pitch_min = 1

    assert_true(Pitch(-2).isUnderMin(config))
    assert_true(Pitch(1.0 - 1e-5).isUnderMin(config))

    assert_false(Pitch(1.5).isUnderMin(config))
    assert_false(Pitch(1.0).isUnderMin(config))

def test_pitch_isLeft():

    config = Config()
    config.pitch_neutral = 1

    assert_true(Pitch(1.5).isLeft(config))
    assert_true(Pitch(1.0).isLeft(config))

    assert_false(Pitch(1.0 - 1e-5).isLeft(config))
    assert_false(Pitch(-2).isLeft(config))

def test_pitch_isRight():

    config = Config()
    config.pitch_neutral = 1

    assert_true(Pitch(1.0 - 1e-5).isRight(config))
    assert_true(Pitch(-2).isRight(config))

    assert_false(Pitch(1.5).isRight(config))
    assert_false(Pitch(1.0).isRight(config))