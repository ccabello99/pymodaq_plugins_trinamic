"""
LECO Director instrument plugin are to be used to communicate (and control) remotely real
instrument plugin through TCP/IP using the LECO Protocol

For this to work a coordinator must be instantiated can be done within the dashboard or directly
running: `python -m pyleco.coordinators.coordinator`

"""
"""
LECO Director instrument plugin are to be used to communicate (and control) remotely real
instrument plugin through TCP/IP using the LECO Protocol

For this to work a coordinator must be instantiated can be done within the dashboard or directly
running: `python -m pyleco.coordinators.coordinator`

"""

from typing import Union, Optional

from pymodaq.control_modules.move_utility_classes import (DAQ_Move_base, comon_parameters_fun, main,
                                                          DataActuatorType, DataActuator)
from pymodaq.control_modules.thread_commands import ThreadStatus, ThreadStatusMove

from pymodaq_utils.utils import ThreadCommand
from pymodaq_utils.utils import find_dict_in_list_from_key_val
from pymodaq_utils.serialize.factory import SerializableFactory
from pymodaq_gui.parameter import Parameter

from pymodaq.utils.leco.leco_director import (LECODirector, leco_parameters, DirectorCommands,
                                              DirectorReceivedCommands)
from pymodaq.utils.leco.director_utils import ActuatorDirector
from pymodaq_plugins_trinamic.resources.extended_publisher import ExtendedPublisher

from pymodaq_utils.logger import set_logger, get_module_name
import numpy as np
import json

logger = set_logger(get_module_name(__file__))


class DAQ_Move_TrinamicLECO(LECODirector, DAQ_Move_base):
    """A control module, which in the dashboard, allows to control a remote Move module.

        ================= ==============================
        **Attributes**      **Type**
        *command_server*    instance of Signal
        *x_axis*            1D numpy array
        *y_axis*            1D numpy array
        *data*              double precision float array
        ================= ==============================

        See Also
        --------
        utility_classes.DAQ_TCP_server
    """
    settings: Parameter
    controller: ActuatorDirector
    _axis_names = ['']
    _controller_units = ['']
    _epsilon = 1

    params_client = []  # parameters of a client grabber
    data_actuator_type = DataActuatorType.DataActuator
    params = comon_parameters_fun(axis_names=_axis_names, epsilon=_epsilon) + leco_parameters+ [
        {'title': "Director Units", 'name': 'director_units', 'type': 'list', 'value': 'microsteps', 'limits': ['mm', 'um', 'nm', 'ps', 'deg', 'rad']},
        {'title': 'LECO Logging', 'name': 'leco_log', 'type': 'group', 'children': [
            {'title': 'Publisher Name', 'name': 'publisher_name', 'type': 'str', 'value': ''},
            {'title': 'Proxy Server Address', 'name': 'proxy_address', 'type': 'str', 'value': 'localhost', 'default': 'localhost'}, # Either IP or hostname of LECO proxy server
            {'title': 'Proxy Server Port', 'name': 'proxy_port', 'type': 'int', 'value': 11100, 'default': 11100},
        ]}
        ]

    for param_name in ('multiaxes', 'units', 'epsilon', 'bounds', 'scaling'):
        param_dict = find_dict_in_list_from_key_val(params, 'name', param_name)
        if param_dict is not None:
            param_dict['visible'] = False

    def __init__(self, actor_name: Optional[str], parent=None, params_state=None) -> None:
        DAQ_Move_base.__init__(self, parent=parent,
                               params_state=params_state)
        LECODirector.__init__(self, host=self.settings['host'])

        self.register_rpc_methods((
            self.set_units,  # to set units accordingly to the one of the actor
        ))

        self.register_binary_rpc_methods((
            self.send_position,  # to display the actor position
            self.set_move_done,  # to set the move as done
        ))
        # To distinguish how to encode positions, it needs to now if it deals
        # with a json-accepting or a binary-accepting actuator
        # It is set to False by default. It then use the first received message
        # from the actuator that should contain its position to decide if it
        # need to switch to json.
        self.json = False
        self.data_publisher = None
        self.director_units = None
        self._move_done_sig = False
        self.settings.param('actor_name').setValue(actor_name)

    def ini_stage(self, controller=None):
        """Actuator communication initialization

        Parameters
        ----------
        controller: (object)
            custom object of a PyMoDAQ plugin (Slave case). None if only one actuator by controller
            (Master case)

        Returns
        -------
        info: str
        initialized: bool
            False if initialization failed otherwise True
        """
        actor_name = self.settings["actor_name"]

        if self.is_master:
            self.controller = ActuatorDirector(actor=actor_name, communicator=self.communicator)
            try:
                self.controller.set_remote_name(self.communicator.full_name)  # type: ignore
            except TimeoutError:
                logger.warning("Timeout setting remote name.")
        else:
            self.controller = controller

        self.json = False
        # send a command to the Actor whose name is actor_name to send its settings
        self.controller.get_settings()
        
        # Allow director units to be different than actor units
        self.director_units = self.settings.param('director_units').value()

        # Set initial timeout very large
        self.settings.child('timeout').setValue(1000)

        # Setup data publisher for LECO if data publisher name is set (ideally it should match the LECO actor name)
        publisher_name = self.settings.child('leco_log', 'publisher_name').value()
        proxy_address = self.settings.child('leco_log', 'proxy_address').value()
        proxy_port = self.settings.child('leco_log', 'proxy_port').value()
        if publisher_name == '':
            print("Publisher name is not set ! Set this first and then reinitialize for LECO logging.")
            self.emit_status(ThreadCommand('Update_Status', ["Publisher name is not set ! Set this first and then reinitialize for LECO logging."]))
        else:
            self.data_publisher = ExtendedPublisher(full_name=publisher_name, host=proxy_address, port=proxy_port)
            print(f"Data publisher {publisher_name} initialized for LECO logging")
            self.emit_status(ThreadCommand('Update_Status', [f"Data publisher {publisher_name} initialized for LECO logging"]))        

        info = f"LECODirector: {self._title} is initialized"
        initialized = True
        return info, initialized
    
    def commit_settings(self, param) -> None:
        if param.name() == 'director_units':
            self.director_units = param.value()
        
        self.commit_leco_settings(param=param)

    def move_abs(self, position: float) -> None:
        units = self.director_units
        position = DataActuator(data=position)
        # We will assume that on the actor side, the scaling will always be such that effective units are mm (for linear stages) and deg (for rotation stages)
        # However, we should be able to provide values remotely in whatever units we want (um, mm, ps (for delays), etc.)
        position_metadata = position.value()
        if units == 'mm':
            current_value_metadata = self.current_value.value()
        elif units == 'um':
            position = position * 1e-3
            current_value_metadata = self.current_value.value() / 1e-3
            self.current_value = self.current_value * 1e-3
        elif units == 'nm':
            position = position * 1e-6
            current_value_metadata = self.current_value.value() / 1e-6
            self.current_value = self.current_value * 1e-6
        elif units == 'ps':
            position = position * 0.299792458
            current_value_metadata = self.current_value.value() / 0.299792458
            self.current_value = self.current_value * 0.299792458
        elif units == 'rad':
            position = position * (180 / np.pi)
            current_value_metadata = self.current_value.value() / (180 / np.pi)
            self.current_value = self.current_value * (180 / np.pi)

        position = self.check_bound(position)
        position = self.set_position_with_scaling(position)
        self.target_value = position
        if self.json:
            position = position.value(self.axis_unit)
        self.controller.move_abs(position=position)            
        
        # Tell proxy server we have started a movement
        metadata = {"actuator_metadata": {}}
        metadata['actuator_metadata']['current_actuator_value'] = current_value_metadata
        metadata['actuator_metadata']['final_actuator_value'] = position_metadata
        metadata['actuator_metadata']['units'] = units
        metadata['actuator_metadata']['message_type'] = "move_start"
        metadata['actuator_metadata']['description'] = f"Moving to {position_metadata} {units} from {current_value_metadata} {units}"
        
        if self.data_publisher is not None:
            serial_number = f"{self.settings.child('settings_client', 'device_manager', 'device_serial_number').value()}_{self.settings.child('settings_client', 'device_manager', 'device_user_id').value()}"
            self.data_publisher.send_data2({self.settings.child('leco_log', 'publisher_name').value():
                                            {'metadata': metadata,
                                            'message_type': 'actuator',
                                            'serial_number': serial_number}})

    def move_rel(self, position: float) -> None:
        units = self.director_units
        position = DataActuator(data=position)

        # We will assume that on the actor side, the scaling will always be such that effective units are mm (for linear stages) and deg (for rotation stages)
        # However, we should be able to provide values remotely in whatever units we want (um, mm, ps (for delays), etc.)
        position_metadata = position.value()
        if units == 'mm':
            current_value_metadata = self.current_value.value()
        elif units == 'um':
            position = position * 1e-3
            current_value_metadata = self.current_value.value() / 1e-3
            self.current_value = self.current_value * 1e-3
        elif units == 'nm':
            position = position * 1e-6
            current_value_metadata = self.current_value.value() / 1e-6
            self.current_value = self.current_value * 1e-6
        elif units == 'ps':
            position = position * 0.299792458
            current_value_metadata = self.current_value.value() / 0.299792458
            self.current_value = self.current_value * 0.299792458
        elif units == 'rad':
            position = position * (180 / np.pi)
            current_value_metadata = self.current_value.value() / (180 / np.pi)
            self.current_value = self.current_value * (180 / np.pi)

        position = self.check_bound(self.current_value + position) - self.current_value  # type: ignore  # noqa
        self.target_value = position + self.current_value

        position = self.set_position_relative_with_scaling(position)
        if self.json:
            position = position.value(self.axis_unit)
        self.controller.move_rel(position=position)
        
        # Tell proxy server we have started a movement
        metadata = {"actuator_metadata": {}}
        metadata['actuator_metadata']['current_actuator_value'] = current_value_metadata
        metadata['actuator_metadata']['final_actuator_value'] = current_value_metadata + position_metadata
        metadata['actuator_metadata']['units'] = units
        metadata['actuator_metadata']['message_type'] = "move_start"
        metadata['actuator_metadata']['description'] = f"Moving to {current_value_metadata + position_metadata} {units} from {current_value_metadata} {units}"
        
        if self.data_publisher is not None:
            serial_number = f"{self.settings.child('settings_client', 'device_manager', 'device_serial_number').value()}_{self.settings.child('settings_client', 'device_manager', 'device_user_id').value()}"
            self.data_publisher.send_data2({self.settings.child('leco_log', 'publisher_name').value():
                                            {'metadata': metadata,
                                            'message_type': 'actuator',
                                            'serial_number': serial_number}})

    def move_home(self):
        units = self.director_units
        self.target_value = 0
        if units == 'mm':
            current_value_metadata = self.current_value.value()        
        if units == 'um':
            current_value_metadata = self.current_value.value() / 1e-3
            self.current_value = self.current_value * 1e-3            
        elif units == 'nm':
            current_value_metadata = self.current_value.value() / 1e-6
            self.current_value = self.current_value * 1e-6
        elif units == 'ps':
            current_value_metadata = self.current_value.value() / 0.299792458
            self.current_value = self.current_value * 0.299792458
        elif units == 'rad':
            current_value_metadata = self.current_value.value() / (180 / np.pi)
            self.current_value = self.current_value * (180 / np.pi)
        # Tell proxy server we have started a movement
        metadata = {"actuator_metadata": {}}
        metadata['actuator_metadata']['current_actuator_value'] = current_value_metadata
        metadata['actuator_metadata']['final_actuator_value'] = 0
        metadata['actuator_metadata']['units'] = units
        metadata['actuator_metadata']['message_type'] = "move_start"
        metadata['actuator_metadata']['description'] = f"Moving to {0} {units} from {current_value_metadata} {units}"
        
        if self.data_publisher is not None:
            serial_number = f"{self.settings.child('settings_client', 'device_manager', 'device_serial_number').value()}_{self.settings.child('settings_client', 'device_manager', 'device_user_id').value()}"
            self.data_publisher.send_data2({self.settings.child('leco_log', 'publisher_name').value():
                                            {'metadata': metadata,
                                            'message_type': 'actuator',
                                            'serial_number': serial_number}})
        self.controller.move_home()

    def get_actuator_value(self) -> DataActuator:
        """ Get the current hardware value """
        self.controller.set_remote_name(self.communicator.full_name)  # to ensure communication
        self.controller.get_actuator_value()
        return self._current_value

    def stop_motion(self) -> None:
        self.controller.stop_motion()
        units = self.director_units
        while self._move_done_sig == False:
            pass # Wait to emit metadata until move done signal confirmed
        if units == 'mm':
            current_value_metadata = self.current_value.value()        
        elif units == 'um':
            current_value_metadata = self.current_value.value() / 1e-3
            self.current_value = self.current_value * 1e-3            
        elif units == 'nm':
            current_value_metadata = self.current_value.value() / 1e-6
            self.current_value = self.current_value * 1e-6
        elif units == 'ps':
            current_value_metadata = self.current_value.value() / 0.299792458
            self.current_value = self.current_value * 0.299792458
        elif units == 'rad':
            current_value_metadata = self.current_value.value() / (180 / np.pi)
            self.current_value = self.current_value * (180 / np.pi)
        # Tell proxy server we have stopped
        metadata = {"actuator_metadata": {}}
        metadata['actuator_metadata']['current_actuator_value'] = current_value_metadata
        metadata['actuator_metadata']['final_actuator_value'] = None
        metadata['actuator_metadata']['units'] = units
        metadata['actuator_metadata']['message_type'] = "move_stop"
        metadata['actuator_metadata']['description'] = f"Stopped at {current_value_metadata} {units}"
        
        if self.data_publisher is not None:
            serial_number = f"{self.settings.child('settings_client', 'device_manager', 'device_serial_number').value()}_{self.settings.child('settings_client', 'device_manager', 'device_user_id').value()}"
            self.data_publisher.send_data2({self.settings.child('leco_log', 'publisher_name').value():
                                            {'metadata': metadata,
                                            'message_type': 'actuator',
                                            'serial_number': serial_number}})

    # Methods accessible via remote calls
    def _set_position_value(
        self, position: Union[str, float, None], additional_payload=None
    ) -> DataActuator:

        # This is the first received message, if position is set then
        # it's included in the json payload and the director should
        # usejson
        if position is not None:
            pos = DataActuator(data=position)
            self.json = True
        elif additional_payload:
            pos = SerializableFactory().get_apply_deserializer(additional_payload[0])
        else:
            raise ValueError("No position given")
        pos = self.get_position_with_scaling(pos)  # type: ignore
        self._current_value = pos
        return pos

    def send_position(self, position: Union[str, float, None], additional_payload=None) -> None:
        pos = self._set_position_value(position=position, additional_payload=additional_payload)
        self.emit_status(ThreadCommand(ThreadStatusMove.GET_ACTUATOR_VALUE, pos))

    def set_move_done(self, position: Union[str, float, None], additional_payload=None) -> None:
        pos = self._set_position_value(position=position, additional_payload=additional_payload)
        self.emit_status(ThreadCommand(ThreadStatusMove.MOVE_DONE, pos))
        self._move_done_sig = True
        # Tell proxy server that the move is done
        current_value = self.current_value
        units = self.director_units
        metadata = {"actuator_metadata": {}}
        metadata['actuator_metadata']['current_actuator_value'] = current_value.value()
        metadata['actuator_metadata']['final_actuator_value'] = None
        metadata['actuator_metadata']['units'] = units
        metadata['actuator_metadata']['message_type'] = "move_done"
        metadata['actuator_metadata']['description'] = "Move done !"
        
        if self.data_publisher is not None:
            serial_number = f"{self.settings.child('settings_client', 'device_manager', 'device_serial_number').value()}_{self.settings.child('settings_client', 'device_manager', 'device_user_id').value()}"
            self.data_publisher.send_data2({self.settings.child('leco_log', 'publisher_name').value():
                                            {'metadata': metadata,
                                            'message_type': 'actuator',
                                            'serial_number': serial_number}})

    def set_units(self, units: str, additional_payload=None) -> None:
        if units not in self.axis_units:
            self.axis_units.append(units)
        self.axis_unit = units

    def set_settings(self, settings: bytes):
        """ Get the content of the actor settings to pe populated in this plugin
        'settings_client' parameter

        Then set the plugin units from this information"""
        super().set_settings(settings)
        self.axis_unit = self.settings['settings_client', 'units']

    def close(self) -> None:
        """ Clear the content of the settings_clients setting"""
        super().close()

if __name__ == '__main__':
    main(__file__, init=False)
