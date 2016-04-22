from multiprocessing import Value,Lock

class Counter(object):
    def __init__(self, initval=0):
        self.val = Value('i', initval)

    def incrementAndGet(self):
        with self.val.get_lock():
            self.val.value += 1
            return self.val.value

    def value(self):
        with self.val.get_lock():
            return self.val.value