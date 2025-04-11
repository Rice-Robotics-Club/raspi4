import rclpy
from rclpy.node import Node

class NoPhasesError(Exception):
    pass

class PhaseController:
    def __init__(self, parent: Node, interval: float, phases=[]):
        self._parent = parent
        self._current_phase = -1
        self._phases = phases
        self._interval = interval
        self._timer = None
        
    def start(self):
        if self._current_phase == -1:
            raise NoPhasesError(message="PhaseController empty")
        
        self._timer = self._parent.create_timer(self._interval, self._exec)
    
    def add_phase(self, phase: tuple[str, function, function]):
        if len(self._phases) == 0:
            self._current_phase += 1
        
        self._phases.append(phase)
        return self
    
    @property
    def current_phase(self):
        return self._phases[self._current_phase]
    
    def _exec(self):
        if self._current_phase < 0 or self._current_phase >= len(self._phases):
            self._current_phase = 0
            self._timer.cancel()
        
        phase = self._phases[self._current_phase]
        phase.func()
        
        if phase.end_condition():
            self._current_phase += 1