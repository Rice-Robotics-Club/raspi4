from math import pi, sqrt, asin, asin, acos, cos, sin, atan
import sys
import typing
import numpy as np
from numpy.typing import NDArray


class FKController:
    def __init__(self, a1: float, a2: float, a3: float, l1: float, l2: float):
        self.legs = [
            [0.0, 0.0],
            [0.0, 0.0],
            [0.0, 0.0],
            [0.0, 0.0],
        ]
        self.leg1 = 0
        self.leg2 = 0

        self.link1_a_x = 0
        self.link1_a_y = 0
        self.link1_a = sqrt(self.link1_a_x**2 + self.link1_a_y)
        self.link1_a_angle = atan(self.link1_a_y / self.link1_a_x)
        self.link1_b = 0
        self.link1_c = 0
        self.link1_d = 0

        self.link2_a = self.leg1
        self.link2_b = 0

    def solve(self, vec: NDArray) -> NDArray:
        m1, m2 = tuple(vec.tolist())

        th2 = 180 - m2
        th6 = 180 - self.link1_a_angle - m1
        l4 = (
            self.link1_a**2
            + self.link1_b**2
            - 2 * self.link1_a * self.link1_b * cos(th6)
        )
        l4_sqrt = sqrt(l4)
        th5 = acos(
            (self.link1_c**2 - self.link1_d**2 - l4)
            / (2 * self.link1_d * l4_sqrt)
        )
        th7 = asin((self.link1_b * sin(th6)) / l4_sqrt)
        th4 = th7 - self.link1_a_angle
        th1 = th4 + th5

        e_x = self.leg1 * cos(th1) - self.leg2 * cos(th2)
        e_y = self.leg1 * sin(th1) + self.leg2 * sin(th2)

        return (e_x, e_y)

    def solve_leg(
        self, position: list[float, float], leg: int
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
        controller = FKController(1.6, 1.0, 0.6, 2.8, 1.8)
        print(controller.solve([x, y, z]))
    else:
        print("needs 3 arguments")
