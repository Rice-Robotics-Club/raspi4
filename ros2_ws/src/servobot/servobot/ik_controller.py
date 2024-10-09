import math
import sys
import typing


class IKController:
    def __init__(self, a1: float, a2: float, a3: float, l1: float, l2: float):
        self.a1 = a1
        self.a2 = a2
        self.a3 = a3
        self.l1 = l1
        self.l2 = l2

        self.a = self.l1**2 + self.l2**2
        self.b = 2 * self.l1 * self.l2
        self.c = 2 * self.a1
        self.d = self.a1**2
        
        self.mult = 180.0 / math.pi

    def solve(self, x: float, y: float, z: float) -> tuple[float, float, float]:
        x2_y2 = x**2 + y**2
        dist_xy = math.sqrt(x2_y2)
        th1 = math.asin(x / dist_xy) - math.asin(self.a3 / dist_xy)
        th3 = math.acos(
            (
                (z - self.a1) ** 2
                + (math.sqrt(x2_y2 - self.a3**2) - self.a2) ** 2
                - self.a
            )
            / self.b
        )

        a = self.b * math.cos(th3) + self.a
        b = math.sqrt(
            (self.l1 + self.l2 * math.cos(th3)) ** 2 * (a + z * self.c - z**2 - self.d)
        )
        th2 = math.acos((math.sin(th3) * self.l2 * (z - self.a1) + b) / a)

        return (self.mult * th1, self.mult * th2, self.mult * th3)

    def solve_leg(
        self, position: tuple[float, float, float], leg: int
    ) -> tuple[float, float, float]:
        """modifies IK calculation depending on the leg specified:

        0 -> FL
        1 -> FR
        2 -> BL
        3 -> BR

        Args:
            pos (tuple[float, float, float]): position coordinate
            leg (int): leg identifier

        Returns:
            tuple[float, float, float]: angles for servo
        """
        x, y, z = position
        match leg:
            case 0:
                th1, th2, th3 = self.solve(x, y, z)
                return (th1, th2, th3)
            case 1:
                th1, th2, th3 = self.solve(-x, y, z)
                return (th1, th2, th3)
            case 2:
                th1, th2, th3 = self.solve(x, y, -z)
                return (th1, th2, th3)
            case 3:
                th1, th2, th3 = self.solve(-x, y, -z)
                return (th1, th2, th3)


if __name__ == "__main__":
    x, y, z = tuple(map(lambda x: float(x), sys.argv[1:4]))
    controller = IKController(1.6, 1.0, 0.6, 2.8, 1.8)
    print(controller.solve(x, y, z))
