"""Script containing the Flow kernel object for interacting with simulators."""

import warnings

from flow.core.kernel.sim_control import TraCISimControl
from flow.core.kernel.simulation import TraCISimulation
from flow.core.kernel.vehicle import TraCIVehicle
from flow.core.kernel.traffic_light import TraCITrafficLight
from flow.utils.exceptions import FatalFlowError


class Kernel(object):
    """Kernel for abstract function calling across traffic simulator APIs.

    The kernel contains four different subclasses for distinguishing between
    the various components of a traffic simulation.

    * sim_control: controls starting, loading, saving, advancing, and resetting
      a simulation in Flow (see flow/core/kernel/simulation/base.py)

    * simulation: stores simulation-specific information (see
      flow/core/kernel/simulation/base.py)

    * vehicle: stores and regularly updates vehicle-specific information. At
      times, this class is optimized to efficiently collect information from
      the simulator (see flow/core/kernel/vehicle/base.py).

    * traffic_light: stores and regularly updates traffic light-specific
      information (see flow/core/kernel/traffic_light/base.py).

    The above kernel subclasses are designed to support simulator-agnostic 
    state information calling. For example, if you would like to collect 
    the vehicle speed of a specific vehicle, then simply do:

    >>> k = Kernel(simulator="...")  # a kernel for some simulator type
    >>> veh_id = "..."  # some vehicle ID
    >>> speed = k.vehicle.get_speed(veh_id)

    In addition, these subclasses support sending commands to the simulator via
    its API. For example, in order to assign a specific vehicle a target
    acceleration, simply do:

    >>> k = Kernel(simulator="...")  # a kernel for some simulator type
    >>> veh_id = "..."  # some vehicle ID
    >>> k.vehicle.apply_acceleration(veh_id)

    These subclasses can be modified and recycled to support various different
    traffic simulators.
    """

    def __init__(self, sim_interface, sim_addl_params):
        """Instantiate a Flow kernel object.

        Parameters
        ----------
        sim_interface : str
            simulator interface, currently only support "traci"
        sim_addl_params : flow.core.params.SimAddlParams
            additional simulation-specific parameters

        Raises
        ------
        flow.utils.exceptions.FatalFlowError
            if the specified input simulator is not a valid type
        """
        self.kernel_api = None

        if sim_interface == "traci":
            self.sim_control = TraCISimControl(self)
            self.simulation = TraCINetwork(self, sim_addl_params)
            self.vehicle = TraCIVehicle(self, sim_addl_params)
            self.traffic_light = TraCITrafficLight(self)
        else:
            raise FatalFlowError('Simulator type "{}" is not valid.'.
                                 format(sim_interface))

    def pass_api(self, kernel_api):
        """Pass the kernel API to all kernel subclasses."""
        self.kernel_api = kernel_api
        self.sim_control.pass_api(kernel_api)
        self.simulation.pass_api(kernel_api)
        self.vehicle.pass_api(kernel_api)
        self.traffic_light.pass_api(kernel_api)

    def update(self, reset):
        """Update the kernel subclasses after a simulation step.

        This is meant to support optimizations in the performance of some
        simulators. For example, this step allows the vehicle subclass in the
        "traci" simulator uses the ``update`` method to collect and store
        subscribed information.

        Parameters
        ----------
        reset : bool
            specifies whether the simulator was reset in the last simulation
            step
        """
        self.vehicle.update(reset)
        self.traffic_light.update(reset)
        self.simulation.update(reset)
        self.sim_control.update(reset)

    def close(self):
        """Terminate all components within the simulation."""
        self.simulation.close()
        self.sim_control.close()


