from math import pi, sqrt, asin, asin, acos, cos, sin
import sys
import typing
import numpy as np
from numpy.typing import NDArray

class IKController:
    def __init__(self, a1: float, a2: float, a3: float, l1: float, l2: float):
        self.legs = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]
        self.a1 = a1
        self.a2 = a2
        self.a3 = a3
        self.l1 = l1
        self.l2 = l2

        # creates pre defined constants
        self.a = self.l1**2 + self.l2**2
        self.b = 2 * self.l1 * self.l2
        self.c = 2 * self.a1
        self.d = self.a1**2

        self.mult = 180.0 / pi
        
        self.input_trs = np.array([
            [1, 1, 1], 
            [1, 1, 1], 
            [1, 1, -1], 
            [1, 1, -1]
        ])
        
        self.output_trs = np.array([
            [1, -1, -1],
            [1, 1, 1],
            [-1, -1, -1],
            [-1, 1, 1]
        ])

    def solve(self, vec: NDArray) -> NDArray:
        x, y, z = tuple(vec.tolist())
        
        x2_y2 = x**2 + y**2
        dist_xy = sqrt(x2_y2)
        th1 = asin(x / dist_xy) - asin(self.a3 / dist_xy)
        th3 = acos(
            ((z - self.a1) ** 2 + (sqrt(x2_y2 - self.a3**2) - self.a2) ** 2 - self.a)
            / self.b
        )
        a = self.b * cos(th3) + self.a
        b = sqrt((self.l1 + self.l2 * cos(th3)) ** 2 * (a + z * self.c - z**2 - self.d))
        th2 = acos((sin(th3) * self.l2 * (z - self.a1) + b) / a)

        return np.array([self.mult * th1, self.mult * th2, self.mult * th3])

    def solve_leg(
        self, position: list[float, float, float], leg: int
    ) -> list[float, float, float]:
        """modifies IK calculation depending on the leg specified:

        0 -> FL
        1 -> FR
        2 -> BL
        3 -> BR

        should account for out of workspace errors

        Args:
            pos (tuple[float, float, float]): position coordinate
            leg (int): leg identifier

        Returns:
            tuple[float, float, float]: angles for servo
        """
        input_tr: NDArray = self.input_trs[leg]
        output_tr: NDArray = self.output_trs[leg]
        
        return (self.solve(np.array(position) * input_tr) * output_tr).tolist()


if __name__ == "__main__":
    print(len(sys.argv))
    if len(sys.argv) >= 4:
        x, y, z = tuple(map(lambda x: float(x), sys.argv[1:4]))
        controller = IKController(1.6, 1.0, 0.6, 2.8, 1.8)
        print(controller.solve([x, y, z]))
    else:
        print("needs 3 arguments")
