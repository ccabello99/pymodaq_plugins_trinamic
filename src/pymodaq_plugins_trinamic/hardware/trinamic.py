import platform
from pytrinamic.connections import SerialTmclInterface, UsbTmclInterface, ConnectionManager
from pytrinamic.modules import TMCM1311
from serial.tools import list_ports
import time


class TrinamicManager:
    def __init__(self):
        self.ports = None
        self.connections = []

    def probe_tmcl_ports(self):
        self.ports = []
        ports = list_ports.comports()

        for port in ports:
            try:
                # Try opening a connection and sending a TMCL "GetVersion"
                conn = UsbTmclInterface(port.device, datarate=9600)
                self.ports.append(port.device)
                conn.close()
            except Exception as e:
                print(f"✖ No TMCL devices found: ({e})")
        return self.ports

    def connect(self, port):
        try:
            self.interface = UsbTmclInterface(port, datarate=9600)
            self.connections.append(port)
            print(f"Connected to TMCL device at {port}")
        except Exception as e:
            print(f"Failed to connect to TMCL device at {port}: {e}")

    def close(self, port):
        try:
            if port in self.connections:
                self.connections.remove(port)
            else:
                print(f"No connection found for {port}")
        except Exception as e:
            print(f"Failed to close connection: {e}")
        

class TrinamicController:
    def __init__(self, port):
        self.port = port
        self.interface = None
        self.module = None
        self.motor = None
        self.possible_microstep_resolution = ["MicrostepResolutionFullstep",
                                                "MicrostepResolutionHalfstep",
                                                "MicrostepResolution4Microsteps",
                                                "MicrostepResolution8Microsteps",
                                                "MicrostepResolution16Microsteps",
                                                "MicrostepResolution32Microsteps",
                                                "MicrostepResolution64Microsteps",
                                                "MicrostepResolution128Microsteps",
                                                "MicrostepResolution256Microsteps"]
        self.reference_position = 0

    def get_version(self):
        if self.connection:
            try:
                version = self.interface.get_version_string()
                return version
            except Exception as e:
                print(f"Failed to get version: {e}")
        else:
            print("No connection established.")

    def connect_module(self, module_type) -> None:
        self.module = module_type(self.interface)

    def connect_motor(self) -> None:
        self.motor = self.module.motors[0]

    def close(self):
        if self.interface:
            self.interface.close()

    @property 
    def max_current(self):
        return self.motor.drive_settings.max_current
    
    @max_current.setter
    def max_current(self, value):
        self.motor.drive_settings.max_current = value
    
    @property
    def standby_current(self):
        return self.motor.drive_settings.standby_current
    
    @standby_current.setter
    def standby_current(self, value):
        self.motor.drive_settings.standby_current = value
    
    @property
    def boost_current(self):
        return self.motor.drive_settings.boost_current
    
    @boost_current.setter
    def boost_current(self, value):
        self.motor.drive_settings.boost_current = value
    
    @property
    def microstep_resolution(self):
        return self.motor.drive_settings.microstep_resolution
    
    @microstep_resolution.setter
    def microstep_resolution(self, value):
        if value == "Full":
            self.motor.drive_settings.microstep_resolution = self.possible_microstep_resolution[0]
        elif value == "Half":
            self.motor.drive_settings.microstep_resolution = self.possible_microstep_resolution[1]
        elif value == "4":
            self.motor.drive_settings.microstep_resolution = self.possible_microstep_resolution[2]
        elif value == "8":
            self.motor.drive_settings.microstep_resolution = self.possible_microstep_resolution[3]
        elif value == "16":
            self.motor.drive_settings.microstep_resolution = self.possible_microstep_resolution[4]
        elif value == "32":
            self.motor.drive_settings.microstep_resolution = self.possible_microstep_resolution[5]
        elif value == "64":
            self.motor.drive_settings.microstep_resolution = self.possible_microstep_resolution[6]
        elif value == "128":
            self.motor.drive_settings.microstep_resolution = self.possible_microstep_resolution[7]
        elif value == "256":
            self.motor.drive_settings.microstep_resolution = self.possible_microstep_resolution[8]

    @property
    def max_velocity(self):
        return self.motor.linear_ramp.max_velocity
    @max_velocity.setter
    def max_velocity(self, value):
        self.motor.linear_ramp.max_velocity = value
    @property
    def max_acceleration(self):
        return self.motor.linear_ramp.max_acceleration
    @max_acceleration.setter
    def max_acceleration(self, value):
        self.motor.linear_ramp.max_acceleration = value

    @property
    def actual_position(self):
        return self.motor.actual_position
    @property
    def target_position(self):
        return self.motor.target_position
    
    @property
    def actual_velocity(self):
        return self.motor.actual_velocity
    @property
    def target_velocity(self):
        return self.motor.target_velocity
    
    def set_closed_loop_mode(self, value):
        if value:
            self.motor.set_axis_parameter(self.motor.AP.ClosedLoopMode, 1)
            while self.motor.get_axis_parameter(self.motor.AP.CLInitFlag) != 1:
                time.sleep(1)
        else:
            self.motor.set_axis_parameter(self.motor.AP.ClosedLoopMode, 0)
            while self.motor.get_axis_parameter(self.motor.AP.CLInitFlag) != 0:
                time.sleep(1)

    def set_relative_motion(self) -> None:
        self.motor.set_axis_parameter(self.motor.AP.RelativePositioningOption, 1)

    def set_absolute_motion(self) -> None:
        self.motor.set_axis_parameter(self.motor.AP.RelativePositioningOption, 0)
    
    def set_reference_position(self) -> None:
        self.motor.actual_position = 0
    
    def move_to(self, position) -> None:
        self.motor.move_to(position, self.motor.linear_ramp.max_velocity)

    def move_by(self, difference) -> None:
        self.motor.move_by(difference, self.motor.linear_ramp.max_velocity)

    def move_to_reference(self) -> None:
        self.move_to(0, self.motor.linear_ramp.max_velocity)
    
    def stop(self) -> None:
        self.motor.stop()
    
    

    


        
    

    



#device_ports = probe_tmcl_ports()
