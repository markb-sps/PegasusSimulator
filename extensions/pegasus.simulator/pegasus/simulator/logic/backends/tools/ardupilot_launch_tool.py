
# System tools used to launch the Ardu process in the brackground
import os
import tempfile
import subprocess


class ArdupilotLaunchTool:
    """
    A class that manages the start/stop of an Ardupilot SITL process.

    NOTE: The Ardupilot SITL instance must be run with the 'airsim' connect as this is how the application receives and interprets the sim sensor data etc.

    """

    def __init__(self, Ardu_dir, vehicle_id: int = 0, Ardu_model: str = "iris"):
        """Construct the ArduLaunchTool object

        Args:
            Ardu_dir (str): A string with the path to the Ardu-Autopilot directory
            vehicle_id (int): The ID of the vehicle. Defaults to 0.
            Ardu_model (str): The vehicle model. Defaults to "iris".

        """
        # Attribute that will hold the Ardu process once it is running
        self.ardu_process = None

        # The vehicle id (used for the mavlink port open in the system)
        self.vehicle_id = vehicle_id

        # Configurations to whether autostart Ardu (SITL) automatically or have the user launch it manually on another
        # terminal
        self.ardu_dir = Ardu_dir

    def launch(self):
        """
        Method that will launch a Ardu instance with the specified configuration
        """
        launch_str = f"{self.ardu_dir}arducopter -M airsim --sim-port-in 5511 --sim-port-out 5501 --rc-in-port 5505 --serial0=udpclient:127.0.0.1:14550"
        print(f"launch cmd: {launch_str}")
        sim_port_in = 5511
        sim_port_out = 5501
        rc_in_port = 5505

        self.ardu_process = subprocess.Popen([f"{self.ardu_dir}arducopter",
                                              "-Mairsim",
                                              "--sim-port-in", f"{sim_port_in}",
                                              "--sim-port-out", f"{sim_port_out}",
                                              "--rc-in-port", f"{rc_in_port}",
                                              "--serial0=udpclient:127.0.0.1:14550"
                                              # "--defaults=sim/copter.parm,sim/airsim-quadX.parm"
                                              ],
            shell=False,
        )

    def kill_sitl(self):
        """
        Method that will kill a Ardu instance with the specified configuration
        """
        if self.ardu_process is not None:
            self.ardu_process.kill()
            self.ardu_process = None

    def __del__(self):
        """
        If the Ardu process is still running when the Ardu launch tool object is whiped from memory, then make sure
        we kill the Ardu instance so we don't end up with hanged Ardu instances
        """

        # Make sure the Ardu process gets killed
        if self.ardu_process:
            self.kill_sitl()
