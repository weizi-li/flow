"""Script containing the base simulation control kernel class."""


class KernelSimControl(object):
    """Base simulation control kernel.

    The simulation control kernel is responsible for generating the simulation and
    passing to all other kernels the API that they can use to interact with the
    simulation.

    The simulation control kernel is also responsible for advancing, resetting, and
    storing whatever simulation data is relevant.

    All methods in this class are abstract and must be overwritten by other
    child classes.
    """

    def __init__(self, master_kernel):
        """Initialize the simulation control kernel.

        Parameters
        ----------
        master_kernel : flow.core.kernel.Kernel
            the higher level kernel (used to call methods from other
            sub-kernels)
        """
        self.master_kernel = master_kernel
        self.kernel_api = None

    def pass_api(self, kernel_api):
        """Acquire the kernel api that was generated by the simulation control kernel.

        Parameters
        ----------
        kernel_api : any
            an API that may be used to interact with the simulator
        """
        self.kernel_api = kernel_api

    def start_simulation(self, simulation, sim_addl_params):
        """Start a simulation instance.

        simulation : any
            an object or variable that is meant to symbolize the simulation. 
	    For example, in the case of SUMO simulations, this is (string) the 
	    path to the .sumo.cfg file.
        sim_addl_params : flow.core.params.SimAddlParams
            additional simulation-specific parameters
        """
        raise NotImplementedError

    def simulation_step(self):
        """Advance the simulation by one step.

        This is done in most cases by calling a relevant simulator API method.
        """
        raise NotImplementedError

    def update(self, reset):
        """Update the internal attributes of the simulation kernel from a simulator.

        Any update operations are meant to support ease of simulation in
        current and future steps.

        Parameters
        ----------
        reset : bool
            specifies whether the simulator was reset in the last simulation
            step
        """
        raise NotImplementedError

    def check_collision(self):
        """Determine if a collision occurred in the last time step.

        Returns
        -------
        bool
            True if collision occurred, False otherwise
        """
        raise NotImplementedError

    def close(self):
        """Close the current simulation control instance."""
        raise NotImplementedError



