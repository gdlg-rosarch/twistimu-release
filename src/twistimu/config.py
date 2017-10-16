class Config:
    """
      Aggregate node configuration parameters
    """

    def __init__(self):

        self.pitch_min = -2
        self.pitch_neutral = 0
        self.pitch_max = 2
        self.roll_min = -0.8
        self.roll_neutral = 0.0
        self.roll_max = 0.8
        self.speed_min = -2
        self.speed_neutral = 0
        self.speed_max = 2
        self.roll_dead_zone = 0.1
        self.pitch_dead_zone = 0.4
        self.max_acceleration = 10
        self.track = 1