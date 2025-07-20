"""
DATE
    19.07.25

DESCRIPTION
    This is the python wrapper on the C++
    `Woctomap` wrapper.
"""

import ctypes

class Octomap:
    def __init__(self):
        pass

    def read_from_text(self, fn: str):
        pass

    def read_from_bt(self, fn: str):
        pass

    def occupied(self, coord: tuple[float]) -> bool:
        return False
    
    def add_point(self, coord: tuple[float], occupied: bool):
        pass