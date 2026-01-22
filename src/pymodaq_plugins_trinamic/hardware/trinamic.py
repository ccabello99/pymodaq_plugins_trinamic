import platform
from qtpy import QtCore
from pytrinamic.connections import SerialTmclInterface, UsbTmclInterface, ConnectionManager
from serial.tools import list_ports
import time

if not hasattr(QtCore, "pyqtSignal"):
    QtCore.pyqtSignal = QtCore.Signal  # type: ignore


class TrinamicManager:
    def __init__(self, baudrate):
        self.devices = None
        self.connections = []
        self.interfaces = []
        self._baudrate = baudrate

    def probe_tmcl_ports(self):
        self.devices = {'ports':[], 'serial_numbers':[]}
        ports = list_ports.comports()

        for port in ports:
            try:
                if platform.system() == 'Windows':
                    self.devices['ports'].append(port.device)
                    self.devices['serial_numbers'].append(port.serial_number)
                else:
                    if port.manufacturer == 'Trinamic Motion Control':
                        self.devices['ports'].append(port.device)
                        self.devices['serial_numbers'].append(port.serial_number)
            except Exception as e:
                pass
        return self.devices

    def connect(self, port):
        try:
            conn = UsbTmclInterface(port, datarate=self._baudrate)
            self.interfaces.append(conn)
            self.connections.append(port)
        except Exception as e:
            print(f"Failed to connect to TMCL device at {port}: {e}")

    def close(self, port):
        try:
            if port in self.connections:
                index = self.connections.index(port)
                self.interfaces[index].close()
                # Clean up both lists
                del self.interfaces[index]
                del self.connections[index]
            else:
                pass
        except Exception as e:
            pass
        

class TrinamicController:
    def __init__(self, device_info):
        self.port = device_info['port']
        self.serial_number = device_info['serial_number']
        self.module = None
        self.motor = None
        self.reference_position = 0
        self.favorite_positions = None

    def connect_module(self, module_type, interface) -> None:
        try:
            self.module = module_type(interface)
        except Exception as e:
            print(f"Failed to connect to module: {e}")

    def connect_motor(self) -> None:
        try:
            self.motor = self.module.motors[0]
        except Exception as e:
            print(f"Failed to connect to motor: {e}")

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
            self.motor.drive_settings.microstep_resolution = self.motor.ENUM.MicrostepResolutionFullstep
        elif value == "Half":
            self.motor.drive_settings.microstep_resolution = self.motor.ENUM.MicrostepResolutionHalfstep
        elif value == "4":
            self.motor.drive_settings.microstep_resolution = self.motor.ENUM.MicrostepResolution4Microsteps
        elif value == "8":
            self.motor.drive_settings.microstep_resolution = self.motor.ENUM.MicrostepResolution8Microsteps
        elif value == "16":
            self.motor.drive_settings.microstep_resolution = self.motor.ENUM.MicrostepResolution16Microsteps
        elif value == "32":
            self.motor.drive_settings.microstep_resolution = self.motor.ENUM.MicrostepResolution32Microsteps
        elif value == "64":
            self.motor.drive_settings.microstep_resolution = self.motor.ENUM.MicrostepResolution64Microsteps
        elif value == "128":
            self.motor.drive_settings.microstep_resolution = self.motor.ENUM.MicrostepResolution128Microsteps
        elif value == "256":
            self.motor.drive_settings.microstep_resolution = self.motor.ENUM.MicrostepResolution256Microsteps

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
                QtCore.QThread.msleep(100)
        else:
            self.motor.set_axis_parameter(self.motor.AP.ClosedLoopMode, 0)
            while self.motor.get_axis_parameter(self.motor.AP.CLInitFlag) != 0:
                QtCore.QThread.msleep(100)

    def set_relative_motion(self) -> None:
        self.motor.set_axis_parameter(self.motor.AP.RelativePositioningOption, 1)

    def set_absolute_motion(self) -> None:
        self.motor.set_axis_parameter(self.motor.AP.RelativePositioningOption, 0)
    
    def set_reference_position(self) -> None:
        self.stop()
        self.motor.set_axis_parameter(self.motor.AP.ActualPosition, 0)
        self.stop()

    def rotate(self, direction: int) -> None:
        self.motor.rotate(direction * self.motor.linear_ramp.max_velocity)
    
    def move_to(self, position) -> None:
        self.motor.move_to(position, self.motor.linear_ramp.max_velocity)

    def move_by(self, difference) -> None:
        self.motor.move_by(difference, self.motor.linear_ramp.max_velocity)

    def move_to_reference(self) -> None:
        self.motor.move_to(0, self.motor.linear_ramp.max_velocity)
    
    def stop(self) -> None:
        self.motor.stop()


class EndStopHitSignal(QtCore.QObject):
    end_stop_hit = QtCore.pyqtSignal(str)
    def __init__(self):
        super().__init__()
        