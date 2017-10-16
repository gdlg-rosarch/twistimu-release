from config import Config

class Pitch:

    def __init__(self, value=0.0):
        self.value = value

    def isInDeadZone(self, config):
        return (self.value > (config.pitch_neutral - config.pitch_dead_zone)) \
         and (self.value < (config.pitch_neutral + config.pitch_dead_zone))

    def isAboveMax(self, config):
        return self.value > config.pitch_max

    def isUnderMin(self, config):
        return self.value < config.pitch_min

    def isLeft(self, config):
        return self.value >= config.pitch_neutral

    def isRight(self, config):
        return self.value < config.pitch_neutral