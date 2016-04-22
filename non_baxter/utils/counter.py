from multiprocessing import Value,Lock

class Counter(object):
    def __init__(self, initval=0):
        self.val = Value('val', initval)
        self.lock = Lock()

    def incrementAndGet(self):
        with self.lock:
            self.val.value += 1
            return self.val.value

    def value(self):
        with self.lock:
            return self.val.value