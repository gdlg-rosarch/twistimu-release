from config import Config

class Roll:

    def __init__(self, value=0.0):
        self.value = value

    def isInDeadZone(self, config):
        return (self.value > (config.roll_neutral - config.roll_dead_zone)) \
         and (self.value < (config.roll_neutral + config.roll_dead_zone))

    def isAboveMax(self, config):
        return self.value > config.roll_max

    def isUnderMin(self, config):
        return self.value < config.roll_min

    def isForward(self, config):
        return self.value >= config.roll_neutral

    def isBackward(self, config):
        return self.value < config.roll_neutral