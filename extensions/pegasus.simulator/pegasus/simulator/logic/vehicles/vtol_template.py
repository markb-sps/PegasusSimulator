import numpy as np

# The vehicle interface
from pegasus.simulator.logic.vehicles.vehicle import Vehicle

# Mavlink interface
from pegasus.simulator.logic.backends.mavlink_backend import MavlinkBackend

# Sensors and dynamics setup
from pegasus.simulator.logic.dynamics import LinearDrag
from pegasus.simulator.logic.thrusters import QuadraticThrustCurve
from pegasus.simulator.logic.sensors import Barometer, IMU, Magnetometer, GPS
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface

class VTOLConfig:
    """
    A data class that is used for configuring a VTOL
    """

    def __init__(self):
        """
        Initialization of the VTOLConfig class
        """

        # Stage prefix of the vehicle when spawning in the world
        self.stage_prefix = "vtol"

        # The USD file that describes the visual aspect of the vehicle (and some properties such as mass and moments of inertia)
        self.usd_file = ""

        # The default thrust curve for a VTOL and dynamics relating to drag
        self.thrust_curve = QuadraticThrustCurve()
        self.drag = LinearDrag([0.50, 0.30, 0.0])

        # The default sensors for a VTOL
        self.sensors = [Barometer(), IMU(), Magnetometer(), GPS()]

        # The backends for actually sending commands to the vehicle. By default use mavlink (with default mavlink configurations)
        # [Can be None as well, if we do not desired to use PX4 with this simulated vehicle]. It can also be a ROS2 backend
        # or your own custom Backend implementation!
        self.backends = [MavlinkBackend()]


class VTOL(Vehicle):
    """VTOL class - It defines a base interface for creating a VTOL
    """
    def __init__(
        self,
        # Simulation specific configurations
        stage_prefix: str = "vtol",
        usd_file: str = "",
        vehicle_id: int = 0,
        # Spawning pose of the vehicle
        init_pos=[0.0, 0.0, 0.07],
        init_orientation=[0.0, 0.0, 0.0, 1.0],
        config=VTOLConfig(),
    ):
        """Initializes the VTOL object

        Args:
            stage_prefix (str): The name the vehicle will present in the simulator when spawned. Defaults to "vtol".
            usd_file (str): The USD file that describes the looks and shape of the vehicle. Defaults to "".
            vehicle_id (int): The id to be used for the vehicle. Defaults to 0.
            init_pos (list): The initial position of the vehicle in the inertial frame (in ENU convention). Defaults to [0.0, 0.0, 0.07].
            init_orientation (list): The initial orientation of the vehicle in quaternion [qx, qy, qz, qw]. Defaults to [0.0, 0.0, 0.0, 1.0].
            config (_type_, optional): _description_. Defaults to VTOLConfig().
        """

        # 1. Initiate the Vehicle object itself
        super().__init__(stage_prefix, usd_file, init_pos, init_orientation)

        # 2. Initialize all the vehicle sensors
        self._sensors = config.sensors
        for sensor in self._sensors:
            sensor.initialize(PegasusInterface().latitude, PegasusInterface().longitude, PegasusInterface().altitude)

        # Add callbacks to the physics engine to update each sensor at every timestep
        # and let the sensor decide depending on its internal update rate whether to generate new data
        self._world.add_physics_callback(self._stage_prefix + "/Sensors", self.update_sensors)

        # 3. Setup the dynamics of the system
        # Get the thrust curve of the vehicle from the configuration
        self._thrusters = config.thrust_curve
        self._drag = config.drag
        # TODO:add lift

        # 4. Save the backend interface (if given in the configuration of the VTOL)
        # and initialize them
        self._backends = config.backends
        for backend in self._backends:
            backend.initialize(self)

        # Add a callback for the MAV state
        self._world.add_physics_callback(self._stage_prefix + "/mav_state", self.update_sim_state)

        # Initialize the ailerons, elevator, and rudder
        self._ailerons = Ailerons()
        self._elevator = Elevator()
        self._rudder = Rudder()

    for sensor in self._sensors:
            sensor_data = sensor.update(self._state, dt)

            # If some data was updated and we have a mavlink backend or ros backend (or other), then just update it
            if sensor_data is not None:
                for backend in self._backends:
                    backend.update_sensor(sensor.sensor_type, sensor_data)

    def update_sim_state(self, dt: float):
        """
        Callback that is used to "send" the current state for each backend being used to control the vehicle. This callback
        is called on every physics step.

        Args:
            dt (float): The time elapsed between the previous and current function calls (s).
        """
        for backend in self._backends:
            backend.update_state(self._state)

    def start(self):
        """
        Intializes the communication with all the backends. This method is invoked automatically when the simulation starts
        """
        for backend in self._backends:
            backend.start()

    def stop(self):
        """
        Signal all the backends that the simulation has stoped. This method is invoked automatically when the simulation stops
        """
        for backend in self._backends:
            backend.stop()

    def update(self, dt: float):
        """
        Method that computes and applies the forces to the VTOL vehicle in simulation based on the control inputs.
        This method must be implemented by a class that inherits this type. This callback is called on every physics step.

        Args:
            dt (float): The time elapsed between the previous and current function calls (s).
        """

        # Get the articulation root of the vehicle
        articulation = self._world.dc_interface.get_articulation(self._stage_prefix)

        # Get the desired angular velocities for each rotor from the first backend (can be mavlink or other) expressed in rad/s
        if len(self._backends) != 0:
            desired_rotor_velocities = self._backends[0].input_reference()
        else:
            desired_rotor_velocities = [0.0 for i in range(self._thrusters._num_rotors)]

        # Input the desired rotor velocities in the thruster model
        self._thrusters.set_input_reference(desired_rotor_velocities)

        # Get the desired forces and torques to apply to the vehicle
        rotor_forces, pusher_force, aileron_forces, elevator_force, rudder_force, yaw_moment = self._thrusters.update(self._state, dt)

        # Apply forces to quad frame rotors
        for i in range(4):
            # Apply the force in the Z-direction on the rotor frame
            self.apply_force([0.0, 0.0, rotor_forces[i]], body_part="/rotor" + str(i))
            # Generate the rotating propeller visual effect
            self.handle_actuation_visual(i, rotor_forces[i], articulation)

        # Apply force to the pusher rotor
        self.apply_force([0.0, 0.0, pusher_force], body_part="/pusher_rotor")

        # Apply forces to the ailerons
        self.apply_force([aileron_forces, 0.0, 0.0], body_part="/ailerons")

        # Apply force to the elevator
        self.apply_force([0.0, elevator_force, 0.0], body_part="/elevator")

        # Apply force to the rudder
        self.apply_force([0.0, 0.0, rudder_force], body_part="/rudder")

        # Apply the torque to the body frame of the vehicle that corresponds to the rolling moment
        self.apply_torque([0.0, 0.0, yaw_moment], "/body")

        # Compute the total linear drag force to apply to the vehicle's body frame
        drag = self._drag.update(self._state, dt)
        self.apply_force(drag, body_part="/body")

        # Call the update methods in all backends
        for backend in self._backends:
            backend.update(dt)
    def handle_actuation_visual(self, actuator_name, control_input: float, articulation):
        """
        Auxiliar method used to set the joint velocity of each actuator (for animation purposes) based on the 
        control input being applied to each joint.

        Args:
            actuator_name (str): The name of the actuator to generate the animation.
            control_input (float): The control input being applied to the actuator.
            articulation (_type_): The articulation group the joints of the actuators belong to.
        """

        # Find the joint corresponding to the actuator
        joint = self._world.dc_interface.find_articulation_dof(articulation, actuator_name)

        # Set the joint velocity for actuation animation
        if control_input > 0:
            self._world.dc_interface.set_dof_velocity(joint, 100)
        elif control_input < 0:
            self._world.dc_interface.set_dof_velocity(joint, -100)
        else:
            self._world.dc_interface.set_dof_velocity(joint, 0)
    def force_and_torques_to_velocities(self, force: float, torque: np.ndarray):
        """
        Auxiliary method used to get the target angular velocities for each actuator, given the total desired thrust [N]
        and torque [Nm] to be applied in the VTOL's body frame.

        Note: This method assumes a quadratic thrust curve. This method will be improved in a future update,
        and a general thrust allocation scheme will be adopted. For now, it is made to work with VTOLs directly.

        Args:
            force (float): The desired total thrust to be applied in the body frame of the VTOL [N].
            torque (np.ndarray): A vector of the torque to be applied in the body frame of the VTOL [Nm].

        Returns:
            list: A list of angular velocities [rad/s] to apply to each actuator to achieve the desired forces and torques.
        """

        # Get the body frame of the VTOL
        body = self._world.dc_interface.get_rigid_body(self._stage_prefix + "/body")

        # Get the actuators of the VTOL
        actuators = [
            self._world.dc_interface.get_rigid_body(self._stage_prefix + "/rotor" + str(i)) for i in range(4)
        ]
        actuators.extend([
            self._world.dc_interface.get_rigid_body(self._stage_prefix + "/pusher_rotor"),
            self._world.dc_interface.get_rigid_body(self._stage_prefix + "/aileron_left"),
            self._world.dc_interface.get_rigid_body(self._stage_prefix + "/aileron_right"),
            self._world.dc_interface.get_rigid_body(self._stage_prefix + "/elevator"),
            self._world.dc_interface.get_rigid_body(self._stage_prefix + "/rudder")
        ])

        # Get the relative positions of the actuators with respect to the body frame of the VTOL (ignoring orientation for now)
        relative_poses = self._world.dc_interface.get_relative_body_poses(body, actuators)

        # Define the allocation matrix
        allocation_matrix = np.zeros((9, 9))

        # Define the first row of the matrix (T [N])
        allocation_matrix[0, :4] = self._thrusters._rotor_constant

        # Define the second and third rows of the matrix (\tau_x [Nm] and \tau_y [Nm])
        allocation_matrix[1, 4:8] = [relative_poses[i].p[1] * self._thrusters._rotor_constant[i] for i in range(4)]
        allocation_matrix[2, 4:8] = [-relative_poses[i].p[0] * self._thrusters._rotor_constant[i] for i in range(4)]

        # Define the fourth row of the matrix (\tau_z [Nm])
        allocation_matrix[3, :4] = [self._thrusters._yaw_moment_coefficient[i] * self._thrusters._rot_dir[i] for i in range(4)]

        # Define the remaining rows for the pusher rotor, ailerons, elevator, and rudder
        allocation_matrix[4, 8] = self._pusher_rotor_constant
        allocation_matrix[5, 4] = self._aileron_constant
        allocation_matrix[5, 6] = -self._aileron_constant
        allocation_matrix[6, 7] = self._elevator_constant
        allocation_matrix[7, 8] = self._rudder_constant

        # Compute the inverse allocation matrix to obtain the angular velocities (squared)
        allocation_inv = np.linalg.pinv(allocation_matrix)
        squared_ang_vel = allocation_inv @ np.array([force, torque[0], torque[1], torque[2], 0.0, 0.0, 0.0, 0.0, 0.0])

        # Ensure that there are no negative values in the target squared angular velocities
        squared_ang_vel[squared_ang_vel < 0] = 0.0

        # Saturate the inputs while preserving their relation to each other by normalizing
        max_thrust_vel_squared = self._thrusters.max_rotor_velocity ** 2
        max_val = np.max(squared_ang_vel)

        if max_val >= max_thrust_vel_squared:
            normalize = np.maximum(max_val / max_thrust_vel_squared, 1.0)
            squared_ang_vel = squared_ang_vel / normalize

        # Compute the angular velocities for each actuator in [rad/s]
        ang_vel = np.sqrt(squared_ang_vel)

        return ang_vel
