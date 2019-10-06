"""Script containing the TraCI simulation kernel class."""

from flow.core.kernel.sim_control import KernelSimControl
from flow.core.util import ensure_dir
import flow.config as config
import traci.constants as tc
import traci
import traceback
import os
import time
import logging
import subprocess
import signal


# Number of retries on restarting SUMO before giving up
RETRIES_ON_ERROR = 10


class TraCISimControl(KernelSimControl):
    """SUMO simulation control kernel.

    Extends flow.core.kernel.sim_control.KernelSimControl
    """

    def __init__(self, master_kernel):
        """Instantiate the SUMO simulation control kernel.

        Parameters
        ----------
        master_kernel : flow.core.kernel.Kernel
            the higher level kernel (used to call methods from other
            sub-kernels)
        """
        KernelSimControl.__init__(self, master_kernel)
        # contains the subprocess.Popen instance used to start traci
        self.sumo_proc = None

    def pass_api(self, kernel_api):
        """See parent class.

        Also initializes subscriptions.
        """
        KernelSimControl.pass_api(self, kernel_api)

        # subscribe some simulation parameters needed to check for entering,
        # exiting, and colliding vehicles
        self.kernel_api.simulation.subscribe([
            tc.VAR_DEPARTED_VEHICLES_IDS, tc.VAR_ARRIVED_VEHICLES_IDS,
            tc.VAR_TELEPORT_STARTING_VEHICLES_IDS, tc.VAR_TIME_STEP,
            tc.VAR_DELTA_T
        ])

    def simulation_step(self):
        """See parent class."""
        self.kernel_api.simulationStep()

    def update(self, reset):
        """See parent class."""
        pass

    def close(self):
        """See parent class."""
        self.kernel_api.close()

    def check_collision(self):
        """See parent class."""
        return self.kernel_api.simulation.getStartingTeleportNumber() != 0

    def start_simulation(self, simulation, sim_addl_params):
        """Start a SUMO simulation control instance.

        This method uses the configuration files created by the simulation class
        to initialize a SUMO instance. It also initializes a traci connection to
        interface with SUMO from Python.
        """
        error = None
        for _ in range(RETRIES_ON_ERROR):
            try:
                # the port number that the SUMO instance will be run on
                port = sim_addl_params.port

                sumo_binary = "sumo-gui" if sim_addl_params.render is True \
                    else "sumo"

                # command used to start SUMO
                sumo_call = [
                    sumo_binary, "-c", simulation.cfg,
                    "--remote-port", str(sim_addl_params.port),
                    "--num-clients", str(sim_addl_params.num_clients),
                    "--step-length", str(sim_addl_params.sim_step)
                ]

                # add step logs (if requested)
                if sim_addl_params.no_step_log:
                    sumo_call.append("--no-step-log")

                # add the lateral resolution of the sublanes (if requested)
                if sim_addl_params.lateral_resolution is not None:
                    sumo_call.append("--lateral-resolution")
                    sumo_call.append(str(sim_addl_params.lateral_resolution))

                # add the emission path to the SUMO command (if requested)
                if sim_addl_params.emission_path is not None:
                    ensure_dir(sim_addl_params.emission_path)
                    emission_out = os.path.join(
                        sim_addl_params.emission_path,
                        "{0}-emission.xml".format(simulation.name))
                    sumo_call.append("--emission-output")
                    sumo_call.append(emission_out)
                else:
                    emission_out = None

                if sim_addl_params.overtake_right:
                    sumo_call.append("--lanechange.overtake-right")
                    sumo_call.append("true")

                # specify a simulation seed (if requested)
                if sim_addl_params.seed is not None:
                    sumo_call.append("--seed")
                    sumo_call.append(str(sim_addl_params.seed))

                if not sim_addl_params.print_warnings:
                    sumo_call.append("--no-warnings")
                    sumo_call.append("true")

                # set the time it takes for a gridlock teleport to occur
                sumo_call.append("--time-to-teleport")
                sumo_call.append(str(int(sim_addl_params.teleport_time)))

                # check collisions at intersections
                sumo_call.append("--collision.check-junctions")
                sumo_call.append("true")

                logging.info(" Starting SUMO on port " + str(port))
                logging.debug(" Cfg file: " + str(simulation.cfg))
                if sim_addl_params.num_clients > 1:
                    logging.info(" Num clients are" +
                                 str(sim_addl_params.num_clients))
                logging.debug(" Emission file: " + str(emission_out))
                logging.debug(" Step length: " + str(sim_addl_params.sim_step))

                # Opening the I/O thread to SUMO
                self.sumo_proc = subprocess.Popen(sumo_call, preexec_fn=os.setsid)

                # wait a small period of time for the subprocess to activate
                # before trying to connect with traci
                if os.environ.get("TEST_FLAG", 0):
                    time.sleep(0.1)
                else:
                    time.sleep(config.SUMO_SLEEP)

                traci_connection = traci.connect(port, numRetries=100)
                traci_connection.setOrder(0)
                traci_connection.simulationStep()

                return traci_connection
            except Exception as e:
                print("Error during start: {}".format(traceback.format_exc()))
                error = e
                self.teardown_sumo()
        raise error

    def teardown_sumo(self):
        """Kill the SUMO subprocess instance."""
        try:
            os.killpg(self.sumo_proc.pid, signal.SIGTERM)
        except Exception as e:
            print("Error during teardown: {}".format(e))

