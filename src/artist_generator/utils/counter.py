from multiprocessing import Value

class Counter(object):
    """
    A process-safe counter providing atomic incrementAndGet() and value() functions
    """
    def __init__(self, initval=0):
        """
        Initialize this counter
        Args:
            initval (int): set the initialize value of the counter
        """
        self.val = Value('i', initval)

    def incrementAndGet(self):
        """
        Atomically increment this counter, and return the new value stored.

        Returns:
            int: The updated value of this counter.
        """
        with self.val.get_lock():
            self.val.value += 1
            return self.val.value

    def value(self):
        """
        Atomically get the current value of this counter.

        Returns:
            int: The current value of this counter.
        """
        with self.val.get_lock():
            return self.val.value