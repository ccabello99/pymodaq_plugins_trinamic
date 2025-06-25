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
import threading

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
        {'title': "Director Units", 'name': 'director_units', 'type': 'list', 'value': 'mm', 'limits': ['mm', 'um', 'nm', 'ps', 'deg', 'rad']},
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

    def __init__(self, name:str, actor_name: Optional[str] = None, coordinator_host: Optional[str] = None, 
                 coordinator_port: Optional[int] = None, proxy_host: Optional[str] = None, 
                 proxy_port: Optional[int] = None, parent=None, params_state=None) -> None:
        DAQ_Move_base.__init__(self, parent=parent,
                               params_state=params_state)
        if coordinator_host and coordinator_port:
            LECODirector.__init__(self, name=name, host=coordinator_host, port=coordinator_port)
        elif coordinator_host and not coordinator_port:
            LECODirector.__init__(self, name=name, host=coordinator_host)
        elif coordinator_port and not coordinator_host:
            LECODirector.__init__(self, name=name, port=coordinator_port)
        else:
            LECODirector.__init__(self, name=name, host=self.settings['host'])

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
        self.name = name
        self.data_publisher = None
        self.director_units = None
        self.actor_name = actor_name
        self._move_done_sig = False
        if actor_name:
            self.settings.param('actor_name').setValue(actor_name)
            self.actor_name = actor_name
        if coordinator_host:
            self.coordinator_host = coordinator_host
        if coordinator_port:
            self.coordinator_port = coordinator_port
        if proxy_host:
            self.proxy_host = proxy_host
        if proxy_port:
            self.proxy_port = proxy_port

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
        else:
            self.controller = controller

        self.json = False
        
        # Allow director units to be different than actor units
        self.director_units = self.settings.param('director_units').value()

        # Set initial timeout very large
        self.settings.child('timeout').setValue(100)

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

        # Make sure we have the current value of the actor with unit conversion
        self.get_actuator_value()

        info = f"LECODirector: {self._title} is initialized"
        initialized = True
        return info, initialized
    
    def connect_actor(self):
        try:
            self.controller.set_remote_name(self.communicator.full_name)  # type: ignore
        except TimeoutError:
            print("Timeout setting remote name.")

        self.controller.get_settings()
    
    def commit_settings(self, param) -> None:
        if param.name() == 'director_units':
            self.director_units = param.value()
        
        self.commit_leco_settings(param=param)

    def move_abs(self, position: float) -> None:
        units = self.director_units
        position = DataActuator(data=position)
        self._move_done_sig = False

        # We will assume that on the actor side, the scaling will always be such that effective units are mm (for linear stages) and deg (for rotation stages)
        # However, we should be able to provide values remotely in whatever units we want (um, rad, ps (for delays), etc.)
        position_metadata = position.value()
        position = self._convert_units_forward(position)
        current_value_metadata = self._convert_units_backward(self.current_value.value())
        self.current_value = self._convert_units_forward(self.current_value)

        position = self.check_bound(position)
        position = self.set_position_with_scaling(position)
        self.target_value = position
        if self.json:
            position = position.value(self.axis_unit)
        self.controller.move_abs(position=position)            
        
        # Tell proxy server we have started a movement  
        self.send_metadata_async(
            current_value=current_value_metadata,
            final_value=position_metadata,
            units=units,
            msg_type="move_start",
            description=f"Moving to {position_metadata} {units} from {current_value_metadata} {units}"
        )

    def move_rel(self, position: float) -> None:
        units = self.director_units
        position = DataActuator(data=position)
        self._move_done_sig = False

        # We will assume that on the actor side, the scaling will always be such that effective units are mm (for linear stages) and deg (for rotation stages)
        # However, we should be able to provide values remotely in whatever units we want (um, mm, ps (for delays), etc.)
        position_metadata = position.value()
        position = self._convert_units_forward(position)
        current_value_metadata = self._convert_units_backward(self.current_value.value())
        self.current_value = self._convert_units_forward(self.current_value)

        position = self.check_bound(self.current_value + position) - self.current_value  # type: ignore  # noqa
        self.target_value = position + self.current_value

        position = self.set_position_relative_with_scaling(position)
        if self.json:
            position = position.value(self.axis_unit)
        self.controller.move_rel(position=position)
        
        # Tell proxy server we have started a movement
        self.send_metadata_async(
            current_value=current_value_metadata,
            final_value=current_value_metadata + position_metadata,
            units=units,
            msg_type="move_start",
            description=f"Moving to {current_value_metadata + position_metadata} {units} from {current_value_metadata} {units}"
        )

    def move_home(self):
        units = self.director_units
        self.target_value = 0
        self._move_done_sig = False
        current_value_metadata = self._convert_units_backward(self.current_value.value())
        self.current_value = self._convert_units_forward(self.current_value)
        # Tell proxy server we have started a movement
        self.send_metadata_async(
            current_value=current_value_metadata,
            final_value=0,
            units=units,
            msg_type="move_start",
            description=f"Moving to {0} {units} from {current_value_metadata} {units}"
        )
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
        # Convert units
        current_value_metadata = self._convert_units_backward(self.current_value.value())
        self.current_value = self._convert_units_forward(self.current_value)
        # Tell proxy server we have stopped   
        self.send_metadata_async(
            current_value=current_value_metadata,
            final_value=None,
            units=units,
            msg_type="move_stop",
            description=f"Stopped at {current_value_metadata} {units}"
        )

    def _convert_units_forward(self, value):
        units = self.director_units
        if units == 'mm':
            return value
        elif units == 'um':
            return value * 1e-3
        elif units == 'nm':
            return value * 1e-6
        elif units == 'ps':
            return value * 0.149896229
        elif units == 'rad':
            return value * (180 / np.pi)
        return value
    def _convert_units_backward(self, value):
        units = self.director_units
        if units == 'mm':
            return value
        elif units == 'um':
            return value / 1e-3
        elif units == 'nm':
            return value / 1e-6
        elif units == 'ps':
            return value / 0.149896229
        elif units == 'rad':
            return value / (180 / np.pi)
        return value
            
    def close(self):
        if self.is_master:
            if self.listener:
                self.listener.close()
            self.listener = None            

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
        units = self.director_units
        current_value_metadata = self._convert_units_backward(self.current_value.value())
        # Tell proxy server that the move is done
        self.send_metadata_async(
            current_value=current_value_metadata,
            final_value=None,
            units=units,
            msg_type="move_done",
            description="Move done !"
        )

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

    def send_metadata_async(self, current_value, final_value, units, msg_type, description):
        if self.data_publisher is None:
            return

        def send():
            try:
                metadata = self.build_actuator_metadata(current_value, final_value, units, msg_type, description)
                serial_number = f"{self.settings.child('settings_client', 'device_manager', 'device_serial_number').value()}_" \
                                f"{self.settings.child('settings_client', 'device_manager', 'device_user_id').value()}"
                publisher_name = self.settings.child('leco_log', 'publisher_name').value()

                payload = {
                    publisher_name: {
                        'metadata': metadata,
                        'message_type': 'actuator',
                        'serial_number': serial_number,
                    }
                }
                self.data_publisher.send_data2(payload)
            except Exception as e:
                print(f"[send_metadata_async] Error: {e}")

        thread = threading.Thread(target=send)
        thread.daemon = True
        thread.start()

    def build_actuator_metadata(self, current_value, final_value, units, msg_type, description):
        return {
            "actuator_metadata": {
                "current_actuator_value": current_value,
                "final_actuator_value": final_value,
                "units": units,
                "message_type": msg_type,
                "description": description,
            }
        }

if __name__ == '__main__':
    main(__file__, init=False)
