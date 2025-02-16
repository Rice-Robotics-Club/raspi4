from math import sqrt, asin, asin, acos, cos, sin
import sys
import typing
import numpy as np
from numpy.typing import NDArray


class FKController:
    def __init__(
        self, a1: float, a2: float, a3: float, a4: float, l1: float, l2: float
    ):
        self.legs = [
            [0.0, 0.0],
            [0.0, 0.0],
            [0.0, 0.0],
            [0.0, 0.0],
        ]
        self.a1 = a1
        self.a2 = a2
        self.a3 = a3
        self.a4 = a4

        self.a1s = self.a1**2
        self.a2s = self.a2**2
        self.a3s = self.a3**2
        self.a4s = self.a4**2

        self.l1 = l1
        self.l2 = l1

    def solve(self, vec: NDArray) -> NDArray:
        m0, m1 = tuple(vec.tolist())

        b1s = self.a1s + self.a2s - 2 * self.a1 * self.a2 * cos(m0)
        b1 = sqrt(b1)

        phi = acos((self.a3s - self.a4s - b1s) / (2 * self.a4 * b1)) + asin(
            (self.a2 * sin(m0)) / b1
        )

        e_x = -self.l1 * cos(phi) + self.l2 * cos(m1)
        e_y = -self.l1 * sin(phi) - self.l2 * sin(m1)

        return np.array([e_x, e_y])


# if __name__ == "__main__":
#     print(len(sys.argv))
#     if len(sys.argv) >= 4:
#         x, y, z = tuple(map(lambda x: float(x), sys.argv[1:4]))
#         controller = FKController(1.6, 1.0, 0.6, 2.8, 1.8)
#         print(controller.solve([x, y, z]))
#     else:
#         print("needs 3 arguments")
