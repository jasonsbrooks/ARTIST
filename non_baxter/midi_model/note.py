class Note(object):
    """
    A Note represents a line of note in a track with the following properties:

    Properties
        pitch: Integer from 0-127 representing note pitch
            Defaults at 60 (C3)
        dur: ???
        tick_dur: Integer representing duration of note in ticks
        start_tick: Integer representing start time of note relative to Song (i.e. 0 is beginning of song)
            Note: NOT relative to Track object, relative to Song object!
        ppqn: ppqn: Integer representing "pulses per quarter note" (i.e. 96 ticks/ quarter note)
    """

    def __init__(self, pitch=60, dur=0, tick_dur=0, start_tick=0, ppqn=100):
        self.pitch = pitch
        self.dur = dur
        self.tick_dur = tick_dur
        self.start_tick = start_tick
        self.ppqn = ppqn
