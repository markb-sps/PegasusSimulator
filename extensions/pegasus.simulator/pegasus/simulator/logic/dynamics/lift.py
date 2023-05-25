"""
| File: linear_lift.py
| Author: Mohammadreza Mousaei (mmousaei@andrew.cmu.edu)
| Description: Computes the forces that should actuate on a rigidbody affected by linear lift
| License: BSD-3-Clause. Copyright (c) 2023. All rights reserved.
"""
import numpy as np
from pegasus.simulator.logic.dynamics.aerodynamics import Aerodynamics
from pegasus.simulator.logic.state import State

class Lift(Aerodynamics):
    """
    Class that implements linear lift computations affecting a rigid body. It inherits the Lift base class.
    """

    def __init__(self, lift_coefficients=[1.5]):
        """
        Receives as input the lift coefficients of the vehicle as a 3x1 vector of constants

        Args:
            lift_coefficients (list[float]): The constant linear lift coefficients used to compute the total lift forces
            affecting the rigid body. The linear lift is given by diag(lx, ly, lz) * [v_x, v_y, v_z] where the velocities
            are expressed in the body frame of the rigid body (using the FRU frame convention).
        """

        # Initialize the base Lift class
        super().__init__()

        # The linear lift coefficients of the vehicle's body frame
        self._lift_coefficients = (lift_coefficients)
        self._air_density = 1.293
        self._wind_surface = 0.1

        # The lift force to apply on the vehicle's body frame
        self._lift_force = np.array([0.0, 0.0, 0.0])

    @property
    def force(self):
        """The lift force to be applied on the body frame of the vehicle

        Returns:
            list: A list with len==3 containing the lift force to be applied on the rigid body according to a FLU body reference
            frame, expressed in Newton (N) [lx, ly, lz]
        """
        return self._lift_force

    def update(self, state: State, dt: float):
        """Method that updates the lift force to be applied on the body frame of the vehicle. The total lift force
        applied on the body reference frame (FLU convention) is given by diag(lx,ly,lz) * R' * v
        where v is the velocity of the vehicle expressed in the inertial frame and R' * v = velocity_body_frame

        Args:
            state (State): The current state of the vehicle.
             dt (float): The time elapsed between the previous and current function calls (s).

        Returns:
            list: A list with len==3 containing the lift force to be applied on the rigid body according to a FLU body reference
        """

        # Get the velocity of the vehicle expressed in the body frame of reference
        body_vel = state.linear_body_velocity

        lift = self._lift_coefficients * self._air_density * self._wind_surface * body_vel[0]**2 / 2

        # Compute the component of the lift force to be applied in the body frame
        self._lift_force = [0, 0, lift]
        return self._lift_force
