import sys
sys.dont_write_bytecode = True

import time

import operations

class SharedMemory:
    def __init__(self):
        self.runtime = 0
        self.is_running = False

sharedmemory = SharedMemory()

measurement = operations.MeasurePpcOxy(0, 10, 0.1, True, sharedmemory)


time.sleep(3)

sharedmemory.is_running = True

time.sleep(10)
