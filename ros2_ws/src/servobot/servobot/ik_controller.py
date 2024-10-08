import math
from enum import Enum


class Leg(Enum):
    FL = 0
    FR = 1
    BL = 2
    BR = 3


class IKController:
    def __init__(self, a1, a2, a3, l1, l2):
        self.a1 = a1
        self.a2 = a2
        self.a3 = a3
        self.l1 = l1
        self.l2 = l2

    def solve(self, x, y, z):
        f = x**2 + y**2
        th1 = math.asin(x / f) - math.asin(self.a3 / f)

        p = math.sqrt(f - self.a3**2)
        g = self.l1**2 + self.l2**2
        h = 2 * self.l1 * self.l2
        th3 = math.acos(((z - self.a1) ** 2 + (p - self.a2) ** 2 - g) / h)

        i = h * math.cos(th3) + g
        j = math.sqrt(
            (self.l1 + self.l2 * math.cos(th3)) ** 2
            * (i + 2 * z * self.a1 - z**2 - self.a1**2)
        )
        th2 = math.acos((math.sin(th3) * self.l2 * (z - self.a1) + j) / i)

        return (th1, th2, th3)

    def solve_leg(self, pos: tuple[float], leg: Leg):
        x, y, z = pos
        match leg:
            case Leg.FL:
                return self.solve(x, y, z)
            case Leg.FR:
                return self.solve(-x, y, z)
            case Leg.BL:
                return self.solve(x, y, -z)
            case Leg.BR:
                return self.solve(-x, y, -z)


import sys

if __name__ == "__main__":
    x, y, z = tuple(map(lambda x: float(x), sys.argv[1:4]))

    controller = IKController(1.6, 1.0, 0.6, 2.8, 1.8)

    print(tuple(map(lambda x: (180.0 / math.pi) * x, controller.solve(x, y, z))))
