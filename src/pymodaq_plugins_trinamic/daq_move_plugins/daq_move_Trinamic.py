import time
import json
import os
import platform
from typing import Union, List, Dict, Tuple
from pymodaq.control_modules.move_utility_classes import (DAQ_Move_base, comon_parameters_fun,
                                                          main, DataActuatorType, DataActuator)

from pymodaq_utils.utils import ThreadCommand  # object used to send info back to the main thread
from pymodaq_gui.parameter import Parameter
from pymodaq_plugins_trinamic.hardware.trinamic import TrinamicManager, TrinamicController, PositionMonitor
from qtpy import QtCore

from pytrinamic.modules import TMCM1311

_last_read_time = 0

class DAQ_Move_Trinamic(DAQ_Move_base):
    """
        * This has been tested with the TMCM-1311 Trinamic stepper motor controller
        * Tested on PyMoDAQ 5.0.6
        * Tested on Python 3.11
        * No additional drivers necessary
    """
    is_multiaxes = False
    _axis_names: Union[List[str], Dict[str, int]] = ['Axis 1']
    _controller_units: Union[str, List[str]] = 'dimensionless' # this actually corresponds to microsteps for our controllers
    data_actuator_type = DataActuatorType.DataActuator

    # Initialize communication at 
    manager = TrinamicManager(baudrate=115200)
    devices = manager.probe_tmcl_ports()

    params = [
                {'title': 'Device Management:', 'name': 'device_manager', 'type': 'group', 'children': [
                    {'title': 'Connected Devices:', 'name': 'connected_devices', 'type': 'list', 'limits': devices['ports']},
                    {'title': 'Selected Device:', 'name': 'selected_device', 'type': 'str', 'value': '', 'readonly': True},
                    {"title": "Device Serial Number", "name": "device_serial_number", "type": "str", "value": "", 'readonly': True},
                    {"title": "Device User ID", "name": "device_user_id", "type": "str", "value": ""},
                    {'title': 'Baudrate:', 'name': 'baudrate', 'type': 'str', 'value': '115200', 'readonly': True}
                ]},
                {'title': 'Closed loop?:', 'name': 'closed_loop', 'type': 'led_push', 'value': False, 'default': False},
                {'title': 'Encoder Settings:', 'name': 'encoder', 'type': 'group', 'children': [
                    {'title': 'Detect Encoder Resolution ?', 'name': 'detect_encoder', 'type': 'bool_push', 'value': False},
                    {'title': 'Encoder Resolution:', 'name': 'encoder_resolution', 'type': 'int', 'value': 1000, 'limits': [1, 1000000]},
                    {'title': 'Encoder Position:', 'name': 'encoder_position', 'type': 'int', 'value': 0, 'readonly': True},
                ]},
                {'title': 'Positioning:', 'name': 'positioning', 'type': 'group', 'children': [
                    {'title': 'Set Reference Position:', 'name': 'set_reference_position', 'type': 'bool_push', 'value': False},
                    {'title': 'Microstep Resolution', 'name': 'microstep_resolution', 'type': 'list', 'value': '256', 'default': '256', 'limits': ['Full', 'Half', '4', '8', '16', '32', '64', '128', '256']},
                ]},
                {'title': 'Motion Control:', 'name': 'motion', 'type': 'group', 'children': [
                    {'title': 'Max Velocity:', 'name': 'max_velocity', 'type': 'int', 'value': 100000, 'limits': [1, 250000]}, # Be careful going to the maximum !
                    {'title': 'Max Acceleration:', 'name': 'max_acceleration', 'type': 'int', 'value': 15000000, 'limits': [1, 30000000]}, # Be careful going to the maximum !
                ]},
                {'title': 'Drive Setting:', 'name': 'drive', 'type': 'group', 'children': [
                    {'title': 'Max Current:', 'name': 'max_current', 'type': 'int', 'value': 75, 'limits': [0, 240]}, # Be careful going to the maximum !
                    {'title': 'Standby Current:', 'name': 'standby_current', 'type': 'int', 'value': 8, 'limits': [0, 240]}, # Be careful going to the maximum !
                    {'title': 'Boost Current:', 'name': 'boost_current', 'type': 'int', 'value': 0, 'limits': [0, 240]}, # Be careful going to the maximum !
                ]},
        ] + comon_parameters_fun(is_multiaxes, axis_names=_axis_names)

    def ini_attributes(self):
        self.controller: TrinamicController = None
        self.user_id = None

    def get_actuator_value(self):
        global _last_read_time
        now = time.time()
        min_interval = 0.15  # seconds
        elapsed = now - _last_read_time
        if elapsed < min_interval:
            QtCore.QThread.msleep(int((min_interval - elapsed) * 1000))
        _last_read_time = time.time()

        pos = DataActuator(data=self.controller.actual_position)
        pos = self.get_position_with_scaling(pos)
        return pos

    def user_condition_to_reach_target(self) -> bool:
        """Adaptive polling to check if the target is reached."""
        max_wait_ms = 1000  # max time to wait
        elapsed = 0
        delay = 10  # start with a small delay in ms

        while elapsed < max_wait_ms:
            if self.controller.motor.get_position_reached():
                return True
            QtCore.QThread.msleep(int(delay))
            elapsed += delay
            delay = min(delay * 1.5, 100)  # back off up to 100ms max delay

        return False  # timeout: assume not reached   

    def close(self):
        """Terminate the communication protocol"""
        port = self.controller.port
        self.controller.port = ''
        self.manager.close(port)

        # Stop any background threads
        if hasattr(self, 'pos_worker'):
            self.pos_worker.stop()
        if hasattr(self, 'pos_thread'):
            self.pos_thread.quit()
            self.pos_thread.wait()
        print("Closed connection to device on port {}".format(port))
        self.controller = None

    def commit_settings(self, param: Parameter):
        """Apply the consequences of a change of value in the detector settings

        Parameters
        ----------
        param: Parameter
            A given parameter (within detector_settings) whose value has been changed by the user
        """
        name = param.name()
        value = param.value()
        if name == 'closed_loop':
            self.controller.set_closed_loop_mode(value)
        elif name == 'max_velocity':
            self.controller.max_velocity = value
        elif name == 'max_acceleration':
            self.controller.max_acceleration = value
        elif name == 'microstep_resolution':
            self.controller.microstep_resolution = value
        elif name == 'set_reference_position':
            if value:
                self.controller.set_reference_position()
                param = self.settings.child('positioning', 'set_reference_position')
                param.setValue(False)
                param.sigValueChanged.emit(param, False)
                self.poll_moving()
        elif name =='max_current':
            self.controller.max_current = value
        elif name =='standby_current':
            self.controller.standby_current = value
        elif name == 'boost_current':
            self.controller.boost_current = value
        elif name == 'detect_encoder':
            # detect encoder resolution
            if value:
                print("Detecting encoder resolution")
                self.emit_status(ThreadCommand('Update_Status', ["Detecting encoder resolution"]))
                self.controller.motor.set_axis_parameter(self.controller.motor.AP.EncoderInitialization, 1)
                timeout = 0
                while self.controller.motor.get_axis_parameter(self.controller.motor.AP.EncoderInitialization) != 2:
                    QtCore.QThread.msleep(100)
                    timeout += 0.1
                    if timeout >= 5:
                        print("Timeout while detecting encoder resolution")
                        self.emit_status(ThreadCommand('Update_Status', ["Timeout while detecting encoder resolution"]))
                        param = self.settings.child('encoder', 'detect_encoder')
                        param.setValue(False)
                        param.sigValueChanged.emit(param, False)
                        return
                print("Encoder resolution = {}".format(self.controller.motor.get_axis_parameter(self.controller.motor.AP.EncoderResolution)))
                self.settings.child('encoder', 'encoder_resolution').setValue(self.controller.motor.get_axis_parameter(self.controller.motor.AP.EncoderResolution))
                param = self.settings.child('encoder', 'detect_encoder')
                param.setValue(False)
                param.sigValueChanged.emit(param, False)
                self.emit_status(ThreadCommand('Update_Status', ["Encoder resolution = {}".format(self.controller.motor.get_axis_parameter(self.controller.motor.AP.EncoderResolution))]))
        elif name == 'encoder_resolution':
            if value > 0:
                self.controller.motor.set_axis_parameter(self.controller.motor.AP.EncoderResolution, value)
                self.settings.child('encoder', 'encoder_position').setValue(0)    
        elif name == 'use_scaling':
            # Update current value in UI
            self.poll_moving()
        elif name == 'device_user_id':
            self.user_id = value
        

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
        self.settings.child('device_manager', 'connected_devices').setLimits(devices['ports'])
        index = devices['ports'].index(self.settings.child('device_manager', 'connected_devices').value())
        device_info = {'port': devices['ports'][index], 'serial_number': devices['serial_numbers'][index]}
        self.user_id = self.settings.child('device_manager', 'device_user_id').value()

        self.ini_stage_init(slave_controller=controller)  # will be useful when controller is slave

        if self.is_master:  # is needed when controller is master
            self.controller = TrinamicController(device_info)
        
        # Establish connection
        self.manager.connect(self.controller.port)
        self.controller.connect_module(TMCM1311, self.manager.interfaces[self.manager.connections.index(self.controller.port)])
        self.controller.connect_motor()
        self.settings.child('device_manager', 'selected_device').setValue(self.controller.port)
        self.settings.child('device_manager', 'device_serial_number').setValue(self.controller.serial_number)

        # Preparing drive settings
        self.controller.max_current = self.settings.child('drive', 'max_current').value()
        self.controller.standby_current = self.settings.child('drive', 'standby_current').value()
        self.controller.boost_current = self.settings.child('drive', 'boost_current').value()

        # Microstep resolution
        self.controller.microstep_resolution = self.settings.child('positioning', 'microstep_resolution').value()

        # Preparing linear ramp settings
        self.controller.max_velocity = self.settings.child('motion', 'max_velocity').value()
        self.controller.max_acceleration = self.settings.child('motion', 'max_acceleration').value()

        # Good initial scaling for testing (~1 degree for rotation and ~1 mm for linear)
        #self.settings.child('scaling', 'use_scaling').setValue(True)
        #self.settings.child('scaling', 'scaling').setValue(1.11111e-5)

        # Hide some useless settings
        self.settings.child('multiaxes').hide()
        self.settings.child('epsilon').hide()

        # Set initial timeout very large
        self.settings.child('timeout').setValue(1000)

        # Start threads for encoder position and limit switch monitoring
        self.start_position_monitoring()

        info = f"Actuator on port {self.controller.port} initialized with baudrate {self.manager._baudrate}"
        initialized = True
        print(info)
        return info, initialized

    def move_abs(self, position: DataActuator):
        """ Move the actuator to the absolute target defined by position

        Parameters
        ----------
        position: (float) value of the absolute target positioning
        """
        position = self.check_bound(position)  #if user checked bounds, the defined bounds are applied here
        self.target_value = position
        position = self.set_position_with_scaling(position)  # apply scaling if the user specified one
        self.controller.set_absolute_motion()
        self.controller.move_to(int(round(position.value())))
        
        self.emit_status(ThreadCommand('Update_Status', ['Moving to absolute position: {}'.format(self.get_position_with_scaling(position).value())]))

    def move_rel(self, position: DataActuator):
        """ Move the actuator to the relative target actuator value defined by position

        Parameters
        ----------
        position: (float) value of the relative target positioning
        """
        position = self.check_bound(self.current_position + position) - self.current_position
        self.target_value = position + self.current_position
        position = self.set_position_relative_with_scaling(position)
        self.controller.set_relative_motion()
        self.controller.move_by(int(round(position.value())))

        self.emit_status(ThreadCommand('Update_Status', ['Moving by: {}'.format(self.get_position_with_scaling(position).value())]))

    def move_home(self):
        """Call the reference method of the controller"""
        self.target_value = 0
        self.controller.move_to_reference()
        self.emit_status(ThreadCommand('Update_Status', ['Moving to zero position']))
        self.poll_moving()

    def stop_motion(self):
      """Stop the actuator and emits move_done signal"""
      self.controller.stop()
      self.move_done()
      self.emit_status(ThreadCommand('Update_Status', ['Stop motion']))

    def start_position_monitoring(self):
        self.pos_thread = QtCore.QThread()
        self.pos_worker = PositionMonitor(self.controller.motor)

        self.pos_worker.moveToThread(self.pos_thread)

        self.pos_thread.started.connect(self.pos_worker.run)
        self.pos_worker.position_updated.connect(self.on_position_update)
        self.pos_worker.finished.connect(self.pos_thread.quit)
        self.pos_worker.finished.connect(self.pos_worker.deleteLater)
        self.pos_thread.finished.connect(self.pos_thread.deleteLater)

        self.pos_thread.start()

    def on_position_update(self, pos: int):
        param = self.settings.child('encoder', 'encoder_position')
        param.setValue(pos)
        param.sigValueChanged.emit(param, pos)


if __name__ == '__main__':
    main(__file__, init=False)
