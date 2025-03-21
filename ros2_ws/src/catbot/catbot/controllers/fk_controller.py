from math import sqrt, asin, asin, acos, cos, sin
import numpy as np
import typing


class FKController:
    def __init__(
        self, a1: float, a2: float, a3: float, a4: float, l1: float, l2: float
    ):
        self.a1 = a1
        self.a2 = a2
        self.a3 = a3
        self.a4 = a4
        self.l1 = l1
        self.l2 = l2

    def forward(self, th1: float, th2: float) -> np.ndarray:
        """Calculates the foot position based on the current angles of the motors.

        Returns:
            np.ndarray: 2D column vector of the foot position
        """
        a1 = self.a1
        a2 = self.a2
        a3 = self.a3
        a4 = self.a4
        l1 = self.l1
        l2 = self.l2

        
        return np.array(
            [
                [
                    -l1
                    * cos(
                        acos(
                            -(
                                -(a1**2)
                                + 2 * a1 * a2 * cos(th1)
                                - a2**2
                                + a3**2
                                - a4**2
                            )
                            / (
                                2
                                * a4
                                * sqrt(a1**2 - 2 * a1 * a2 * cos(th1) + a2**2)
                            )
                        )
                        - asin(
                            a2
                            * sin(th1)
                            / sqrt(a1**2 - 2 * a1 * a2 * cos(th1) + a2**2)
                        )
                    )
                    + l2 * cos(th2)
                ],
                [
                    -l1
                    * sin(
                        acos(
                            -(
                                -(a1**2)
                                + 2 * a1 * a2 * cos(th1)
                                - a2**2
                                + a3**2
                                - a4**2
                            )
                            / (
                                2
                                * a4
                                * sqrt(a1**2 - 2 * a1 * a2 * cos(th1) + a2**2)
                            )
                        )
                        - asin(
                            a2
                            * sin(th1)
                            / sqrt(a1**2 - 2 * a1 * a2 * cos(th1) + a2**2)
                        )
                    )
                    + l2 * sin(th2)
                ],
            ]
        )

    def jacobian(self, th1: float, th2: float) -> np.ndarray:
        """Calculates the Jacobian matrix of the leg based on the current angles of the motors.

        Returns:
            np.ndarray: 2x2 Jacobian matrix
        """
        a1 = self.a1
        a2 = self.a2
        a3 = self.a3
        a4 = self.a4
        l1 = self.l1
        l2 = self.l2

        # it works, and faster than the previous way
        return np.array(
            [
                [
                    l1
                    * (
                        -(
                            -a1
                            * a2**2
                            * sin(th1) ** 2
                            / (a1**2 - 2 * a1 * a2 * cos(th1) + a2**2)
                            ** (3 / 2)
                            + a2
                            * cos(th1)
                            / sqrt(a1**2 - 2 * a1 * a2 * cos(th1) + a2**2)
                        )
                        / sqrt(
                            -(a2**2)
                            * sin(th1) ** 2
                            / (a1**2 - 2 * a1 * a2 * cos(th1) + a2**2)
                            + 1
                        )
                        - (
                            a1
                            * a2
                            * sin(th1)
                            / (
                                a4
                                * sqrt(a1**2 - 2 * a1 * a2 * cos(th1) + a2**2)
                            )
                            + a1
                            * a2
                            * (
                                -(a1**2)
                                + 2 * a1 * a2 * cos(th1)
                                - a2**2
                                + a3**2
                                - a4**2
                            )
                            * sin(th1)
                            / (
                                2
                                * a4
                                * (a1**2 - 2 * a1 * a2 * cos(th1) + a2**2)
                                ** (3 / 2)
                            )
                        )
                        / sqrt(
                            1
                            - (
                                -(a1**2)
                                + 2 * a1 * a2 * cos(th1)
                                - a2**2
                                + a3**2
                                - a4**2
                            )
                            ** 2
                            / (
                                4
                                * a4**2
                                * (a1**2 - 2 * a1 * a2 * cos(th1) + a2**2)
                            )
                        )
                    )
                    * sin(
                        acos(
                            -(
                                -(a1**2)
                                + 2 * a1 * a2 * cos(th1)
                                - a2**2
                                + a3**2
                                - a4**2
                            )
                            / (
                                2
                                * a4
                                * sqrt(a1**2 - 2 * a1 * a2 * cos(th1) + a2**2)
                            )
                        )
                        - asin(
                            a2
                            * sin(th1)
                            / sqrt(a1**2 - 2 * a1 * a2 * cos(th1) + a2**2)
                        )
                    ),
                    -l2 * sin(th2),
                ],
                [
                    -l1
                    * (
                        -(
                            -a1
                            * a2**2
                            * sin(th1) ** 2
                            / (a1**2 - 2 * a1 * a2 * cos(th1) + a2**2)
                            ** (3 / 2)
                            + a2
                            * cos(th1)
                            / sqrt(a1**2 - 2 * a1 * a2 * cos(th1) + a2**2)
                        )
                        / sqrt(
                            -(a2**2)
                            * sin(th1) ** 2
                            / (a1**2 - 2 * a1 * a2 * cos(th1) + a2**2)
                            + 1
                        )
                        - (
                            a1
                            * a2
                            * sin(th1)
                            / (
                                a4
                                * sqrt(a1**2 - 2 * a1 * a2 * cos(th1) + a2**2)
                            )
                            + a1
                            * a2
                            * (
                                -(a1**2)
                                + 2 * a1 * a2 * cos(th1)
                                - a2**2
                                + a3**2
                                - a4**2
                            )
                            * sin(th1)
                            / (
                                2
                                * a4
                                * (a1**2 - 2 * a1 * a2 * cos(th1) + a2**2)
                                ** (3 / 2)
                            )
                        )
                        / sqrt(
                            1
                            - (
                                -(a1**2)
                                + 2 * a1 * a2 * cos(th1)
                                - a2**2
                                + a3**2
                                - a4**2
                            )
                            ** 2
                            / (
                                4
                                * a4**2
                                * (a1**2 - 2 * a1 * a2 * cos(th1) + a2**2)
                            )
                        )
                    )
                    * cos(
                        acos(
                            -(
                                -(a1**2)
                                + 2 * a1 * a2 * cos(th1)
                                - a2**2
                                + a3**2
                                - a4**2
                            )
                            / (
                                2
                                * a4
                                * sqrt(a1**2 - 2 * a1 * a2 * cos(th1) + a2**2)
                            )
                        )
                        - asin(
                            a2
                            * sin(th1)
                            / sqrt(a1**2 - 2 * a1 * a2 * cos(th1) + a2**2)
                        )
                    ),
                    l2 * cos(th2),
                ],
            ]
        )
