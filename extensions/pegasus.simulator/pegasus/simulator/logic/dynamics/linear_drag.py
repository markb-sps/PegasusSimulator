"""
| File: linear_drag.py
| Author: Marcelo Jacinto (marcelo.jacinto@tecnico.ulisboa.pt)
| Description: Computes the forces that should actuate on a rigidbody affected by linear drag
| License: BSD-3-Clause. Copyright (c) 2023, Marcelo Jacinto. All rights reserved.
"""
import numpy as np
from pegasus.simulator.logic.dynamics.aerodynamics import Aerodynamics
from pegasus.simulator.logic.state import State
from scipy.io import loadmat
from scipy.interpolate import interp1d

class LinearDrag(Aerodynamics):
    """
    Class that implements linear drag computations afftecting a rigid body. It inherits the Drag base class.
    """

    def __init__(self, drag_coefficients=[0.0, 0.0, 0.0]):
        """
        Receives as input the drag coefficients of the vehicle as a 3x1 vector of constants

        Args:
            drag_coefficients (list[float]): The constant linear drag coefficients to used to compute the total drag forces
            affecting the rigid body. The linear drag is given by diag(dx, dy, dz) * [v_x, v_y, v_z] where the velocities
            are expressed in the body frame of the rigid body (using the FRU frame convention).
        """

        # Initialize the base Drag class
        super().__init__()

        # The linear drag coefficients of the vehicle's body frame
        # self._drag_coefficients = np.diag(drag_coefficients)
        self._drag_coefficients = (drag_coefficients)
        self._air_density = 1.293
        self._reference_area = self._wind_surface / 10
        # The drag force to apply on the vehicle's body frame
        self._drag_force = np.array([0.0, 0.0, 0.0])

    @property
    def force(self):
        """The drag force to be applied on the body frame of the vehicle

        Returns:
            list: A list with len==3 containing the drag force to be applied on the rigid body according to a FLU body reference
            frame, expressed in Newton (N) [dx, dy, dz]
        """
        return self._drag_force
    
    def get_cd(self, alpha):
        data = loadmat('/home/honda/mohammad/C_d.mat')  # Load .mat file
        cd_data = data['C_d'].ravel()  # Assuming 'C_l' is a 1D array in the .mat file

        alpha_points = np.concatenate([np.arange(-8.5, 14, 0.25), [14.5, 14.75, 15]])

    
        # Create an interpolation function based on the input data
        f = interp1d(alpha_points, cd_data, kind='nearest', fill_value='extrapolate')

        # Use the function to interpolate the input alpha
        cd = f(alpha)

        return cd

    def update(self, state: State, pitch, dt: float):
        """Method that updates the drag force to be applied on the body frame of the vehicle. The total drag force
        applied on the body reference frame (FLU convention) is given by diag(dx,dy,dz) * R' * v
        where v is the velocity of the vehicle expressed in the inertial frame and R' * v = velocity_body_frame

        Args:
            state (State): The current state of the vehicle.
             dt (float): The time elapsed between the previous and current function calls (s).

        Returns:
            list: A list with len==3 containing the drag force to be applied on the rigid body according to a FLU body reference
        """

        # Get the velocity of the vehicle expressed in the body frame of reference
        body_vel = state.linear_body_velocity
        
        with open('/home/honda/Documents/wind_surface.txt', 'r') as f:
            content = f.read()
        self._wind_surface = float(content)
        # self._reference_area = 1.012642281 # from Omega_trim = 60% = 663.94 and V_trim = 15 m/s^2
        self._reference_area = 0.176302695 # from Omega_trim = 25% = 276.64 and V_trim = 15 m/s^2
        # self._drag_coefficients[0] = self.get_cd(pitch)

        drag = self._drag_coefficients[0] * self._air_density * self._wind_surface *(body_vel[0]**2) / 2
        # print("self._drag_coefficients[0] = ", self._drag_coefficients[0])
        # print("self._air_density = ", self._air_density)
        # print(" = ", drag)
        # print(" = ", drag)
        # Compute the component of the drag force to be applied in the body frame
        self._drag_force = [-1*drag, 0, 0]
        # self._drag_force[0] = -drag
        # self._drag_force = -np.dot(self._drag_coefficients, np.multiply(np.square(body_vel), np.sign(body_vel)))
        return self._drag_force
