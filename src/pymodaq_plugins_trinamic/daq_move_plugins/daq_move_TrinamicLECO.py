"""
LECO Director instrument plugin are to be used to communicate (and control) remotely real
instrument plugin through TCP/IP using the LECO Protocol

For this to work a coordinator must be instantiated can be done within the dashboard or directly
running: `python -m pyleco.coordinators.coordinator`

"""

from typing import Union, List, Dict, Tuple

from pymodaq.control_modules.move_utility_classes import (DAQ_Move_base, comon_parameters_fun, main,
                                                          DataActuatorType, DataActuator)

from pymodaq_utils.utils import ThreadCommand
from pymodaq_gui.parameter import Parameter
from pymodaq_gui.utils.utils import mkQApp

from pymodaq.utils.leco.leco_director import LECODirector, leco_parameters
from pymodaq.utils.leco.director_utils import ActuatorDirector
from pymodaq_utils.serialize.serializer_legacy import DeSerializer

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


    is_multiaxes = False
    _axis_names: Union[List[str], Dict[str, int]] = ['Axis 1']
    data_actuator_type = DataActuatorType.float

    message_list = LECODirector.message_list + ["move_abs", 'move_home', 'move_rel',
                                                'get_actuator_value', 'stop_motion', 'position_is',
                                                'move_done']
    socket_types = ["ACTUATOR"]
    params = comon_parameters_fun(is_multiaxes, axis_names=_axis_names) + leco_parameters


    def __init__(self, parent=None, params_state=None, **kwargs) -> None:
        super().__init__(parent=parent,
                         params_state=params_state, **kwargs)
        self.register_rpc_methods((
            self.set_info,
        ))
        for method in (
            self.set_position,
            self.set_move_done,
            self.set_x_axis,
            self.set_y_axis,
        ):
            self.listener.register_binary_rpc_method(method, accept_binary_input=True)

    def commit_settings(self, param) -> None:
        self.commit_leco_settings(param=param)

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
        actor_name = self.settings.child("actor_name").value()
        self.controller = self.ini_stage_init(  # type: ignore
            old_controller=controller,
            new_controller=ActuatorDirector(actor=actor_name, communicator=self.communicator),
            )
        try:
            self.controller.set_remote_name(self.communicator.full_name)  # type: ignore
        except TimeoutError:
            print("Timeout setting remote name.")  # TODO change to real logging

        # for some reason factor of 1e-3 btwn position values for good responsiveness in this module and original module
        self.settings.child('scaling', 'use_scaling').setValue(True)
        self.settings.child('scaling', 'scaling').setValue(1e-3)
        self.settings.child('scaling').hide()


        # Hide some useless settings
        self.settings.child('multiaxes').hide()
        self.settings.child('epsilon').hide()
        self.settings.child('bounds').hide()
        self.settings.child('units').hide()
        self.settings.child('epsilon').setValue(1)

        self.status.info = "LECODirector"
        self.status.controller = self.controller
        self.status.initialized = True
        return self.status

    def move_abs(self, value: DataActuator) -> None:
        value = self.set_position_with_scaling(value)
        self.controller.move_abs(position=value)
        self.emit_status(ThreadCommand('Update_Status', ['Moving to absolute position: {}'.format(value)]))

    def move_rel(self, value: DataActuator) -> None:
        value = self.set_position_relative_with_scaling(value)
        self.controller.move_rel(position=value)
        self.emit_status(ThreadCommand('Update_Status', ['Moving by: {}'.format(value)]))
        

    def move_home(self):
        self.controller.move_home()

    def get_actuator_value(self) -> DataActuator:
        """
        Get the current hardware position with scaling conversion given by
        `get_position_with_scaling`.

        See Also
        --------
            daq_move_base.get_position_with_scaling, daq_utils.ThreadCommand
        """
        self.controller.set_remote_name(self.communicator.full_name)  # to ensure communication
        self.controller.get_actuator_value()
        return self._current_value

    def stop_motion(self) -> None:
        """
            See Also
            --------
            daq_move_base.move_done
        """
        self.controller.move_rel(position=0)
        self.move_done()

    # Methods accessible via remote calls
    def _set_position_value(
        self, position: Union[str, float, None], additional_payload=None
    ) -> DataActuator:
        if position:
            if isinstance(position, str):
                deserializer = DeSerializer.from_b64_string(position)
                pos = deserializer.dwa_deserialization()
            else:
                pos = DataActuator(data=position)
        elif additional_payload is not None:
            pos = DeSerializer(additional_payload[0]).dwa_deserialization()
        else:
            raise ValueError("No position given")
        pos = self.get_position_with_scaling(pos)  # type: ignore
        self._current_value = pos
        return pos

    def set_position(self, position: Union[str, float, None], additional_payload=None) -> None:
        pos = self._set_position_value(position=position, additional_payload=additional_payload)
        self.emit_status(ThreadCommand('get_actuator_value', [pos]))

    def set_move_done(self, position: Union[str, float, None], additional_payload=None) -> None:
        pos = self._set_position_value(position=position, additional_payload=additional_payload)
        self.emit_status(ThreadCommand('move_done', [pos]))

    def set_x_axis(self, data, label: str = "", units: str = "") -> None:
        raise NotImplementedError("where is it handled?")

    def set_y_axis(self, data, label: str = "", units: str = "") -> None:
        raise NotImplementedError("where is it handled?")


if __name__ == '__main__':
    main(__file__, init=False)