import time
from typing import Union, List, Dict, Tuple
from pymodaq.control_modules.move_utility_classes import (DAQ_Move_base, comon_parameters_fun,
                                                          main, DataActuatorType, DataActuator)

from pymodaq_utils.utils import ThreadCommand  # object used to send info back to the main thread
from pymodaq_gui.parameter import Parameter
from pymodaq_plugins_trinamic.hardware.trinamic import TrinamicManager, TrinamicController
from qtpy import QtCore
from pymodaq_utils.serialize.serializer_legacy import DeSerializer

from pytrinamic.modules import TMCM1311


class DAQ_Move_Trinamic(DAQ_Move_base):
    """
        * This has been tested with the TMCM-1311 Trinamic stepper motor controller
        * Tested on PyMoDAQ 5.0.6
        * Tested on Python 3.11
        * No additional drivers necessary
    """
    is_multiaxes = False
    _axis_names: Union[List[str], Dict[str, int]] = ['Axis 1']
    _controller_units: Union[str, List[str]] = 'um' # this is bullshit for our controller, but keep it for now
    data_actuator_type = DataActuatorType.DataActuator

    manager = TrinamicManager()
    devices = manager.probe_tmcl_ports()

    params = [
                {'title': 'Device Management:', 'name': 'device_manager', 'type': 'group', 'children': [
                    {'title': 'Connected Devices:', 'name': 'connected_devices', 'type': 'list', 'limits': devices},
                    {'title': 'Selected Device:', 'name': 'selected_device', 'type': 'str', 'value': '', 'readonly': True},
                    {'title': 'Baudrate:', 'name': 'baudrate', 'type': 'int', 'value': 9600, 'readonly': True},
                ]},
                {'title': 'Closed loop?:', 'name': 'closed_loop', 'type': 'led_push', 'value': False, 'default': False},
                {'title': 'Positioning:', 'name': 'positioning', 'type': 'group', 'children': [
                    {'title': 'Set Reference Position:', 'name': 'set_reference_position', 'type': 'bool_push', 'value': False},
                    {'title': 'Microstep Resolution', 'name': 'microstep_resolution', 'type': 'list', 'value': '256', 'default': '256', 'limits': ['Full', 'Half', '4', '8', '16', '32', '64', '128', '256']}
                ]},
                {'title': 'Motion Control:', 'name': 'motion', 'type': 'group', 'children': [
                    {'title': 'Max Velocity:', 'name': 'max_velocity', 'type': 'int', 'value': 200000, 'limits': [1, 250000]},
                    {'title': 'Max Acceleration:', 'name': 'max_acceleration', 'type': 'int', 'value': 22000000, 'limits': [1, 30000000]},
                    {'title': 'Stage Type:', 'name': 'stage_type', 'type': 'list', 'value': 'Linear', 'limits': ['Linear', 'Rotary']},
                ]},
                {'title': 'Drive Setting:', 'name': 'drive', 'type': 'group', 'children': [
                    {'title': 'Max Current:', 'name': 'max_current', 'type': 'int', 'value': 150, 'limits': [0, 240]}, 
                    {'title': 'Standby Current:', 'name': 'standby_current', 'type': 'int', 'value': 8, 'limits': [0, 240]},
                    {'title': 'Boost Current:', 'name': 'boost_current', 'type': 'int', 'value': 0, 'limits': [0, 10]},
                ]},
        ] + comon_parameters_fun(is_multiaxes, axis_names=_axis_names)

    def ini_attributes(self):
        self.controller: TrinamicController = None

    def get_actuator_value(self):
        """Get the current value from the hardware with scaling conversion.

        Returns
        -------
        float: The position obtained after scaling conversion.
        """
        # Block (kinda) to avoid reading the position too fast (bad for controller)
        QtCore.QThread.msleep(200)
        pos = DataActuator(data=self.controller.actual_position)
        pos = self.get_position_with_scaling(pos)
        return pos

    def user_condition_to_reach_target(self) -> bool:
        """ Implement a condition for exiting the polling mechanism and specifying that the
        target value has been reached

       Returns
        -------
        bool: if True, PyMoDAQ considers the target value has been reached
        """
        # Block (kinda) until the target position is reached for same reason as above
        QtCore.QThread.msleep(200)
        if self.controller.motor.get_position_reached():
            return True
        else:
            return False        

    def close(self):
        """Terminate the communication protocol"""
        port = self.controller.port
        self.controller.port = ''
        self.manager.close(port)
        print("Closed connection to device on port {}".format(port))
        self.controller = None

    def commit_settings(self, param: Parameter):
        """Apply the consequences of a change of value in the detector settings

        Parameters
        ----------
        param: Parameter
            A given parameter (within detector_settings) whose value has been changed by the user
        """
        if param.name() == 'closed_loop':
            self.controller.set_closed_loop_mode(param.value())
        elif param.name() == 'max_velocity':
            self.controller.max_velocity = param.value()
        elif param.name() == 'max_acceleration':
            self.controller.max_acceleration = param.value()
        elif param.name() == 'microstep_resolution':
            self.controller.microstep_resolution = param.value()
        elif param.name() == 'set_reference_position':
            if param.value():
                self.controller.set_reference_position()
                self.settings.child('positioning', 'set_reference_position').setValue(False)
                self.poll_moving()
        elif param.name() =='max_current':
            self.controller.max_current = param.value()
        elif param.name() =='standby_current':
            self.controller.standby_current = param.value()

    def ini_stage(self, controller=None):
        """Actuator communication initialization

        Parameters
        ----------
        controller: (object)
            custom object of a PyMoDAQ plugin (Slave case). None if only one actuator by controller (Master case)

        Returns
        -------
        info: str
        initialized: bool
            False if initialization failed otherwise True
        """
        # Always get a fresh list on device initialization
        devices = self.manager.probe_tmcl_ports()
        self.settings.child('device_manager', 'connected_devices').setLimits(devices)

        self.ini_stage_init(slave_controller=controller)  # will be useful when controller is slave

        if self.is_master:  # is needed when controller is master
            self.controller = TrinamicController(self.settings.child('device_manager', 'connected_devices').value())
        
        # Establish connection
        self.manager.connect(self.controller.port)
        self.controller.connect_module(TMCM1311, self.manager.interfaces[self.manager.connections.index(self.controller.port)])
        self.controller.connect_motor()
        self.settings.child('device_manager', 'selected_device').setValue(self.controller.port)

        # Preparing drive settings
        self.controller.max_current = self.settings.child('drive', 'max_current').value()
        self.controller.standby_current = self.settings.child('drive', 'standby_current').value()
        self.controller.boost_current = self.settings.child('drive', 'boost_current').value()

        # Microstep resolution
        self.controller.microstep_resolution = self.settings.child('positioning', 'microstep_resolution').value()

        # Preparing linear ramp settings
        self.controller.max_velocity = self.settings.child('motion', 'max_velocity').value()
        self.controller.max_acceleration = self.settings.child('motion', 'max_acceleration').value()

        info = "Actuator on port {} initialized".format(self.controller.port)
        initialized = True
        print(info)
        return info, initialized

    def move_abs(self, value: DataActuator):
        """ Move the actuator to the absolute target defined by value

        Parameters
        ----------
        value: (float) value of the absolute target positioning
        """

        value = self.check_bound(value)  #if user checked bounds, the defined bounds are applied here
        self.target_value = value
        value = self.set_position_with_scaling(value)  # apply scaling if the user specified one
        self.controller.set_absolute_motion()
        self.controller.move_to(int(round(value.value())))
        
        self.emit_status(ThreadCommand('Update_Status', ['Moving to absolute position: {}'.format(value.value())]))

    def move_rel(self, value: DataActuator):
        """ Move the actuator to the relative target actuator value defined by value

        Parameters
        ----------
        value: (float) value of the relative target positioning
        """
        value = self.check_bound(self.current_position + value) - self.current_position
        self.target_value = value + self.current_position
        value = self.set_position_relative_with_scaling(value)
        self.controller.set_relative_motion()
        self.controller.move_by(int(round(value.value())))
        self.emit_status(ThreadCommand('Update_Status', ['Moving by: {}'.format(value.value())]))

    def move_home(self):
        """Call the reference method of the controller"""
        self.controller.move_to_reference()
        self.emit_status(ThreadCommand('Update_Status', ['Moving to zero position']))
        self.poll_moving()

    def stop_motion(self):
      """Stop the actuator and emits move_done signal"""
      self.controller.stop()
      self.move_done()
      self.emit_status(ThreadCommand('Update_Status', ['Stop motion']))


if __name__ == '__main__':
    main(__file__)
