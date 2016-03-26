class Note(object):
    """
    A Note represents a line of note in a track with the following properties:

    Properties
        pitch: Integer from 0-127 representing note pitch
            Defaults at 60 (C3)
            Should -1 represent a rest?
        dur: ??? Float where 1.0 ~ quarter note
        tick_dur: Integer representing duration of note in ticks
        start_tick: Integer representing start time of note relative to Song (i.e. 0 is beginning of song)
            Note: NOT relative to Track object, relative to Song object!
    """

    def __init__(self, pitch=60, dur=0, tick_dur=0, start_tick=0):
        self.pitch = pitch
        self.dur = dur
        self.tick_dur = tick_dur
        self.start_tick = start_tick

    def __repr__(self):
        return "note(pitch=%r, dur=%r, start_tick=%r)" % \
            (self.pitch, self.dur, self.start_tick)
