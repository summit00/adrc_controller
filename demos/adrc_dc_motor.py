import sys
from pathlib import Path

project_root = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(project_root))

from adrc.adrcController import ADRCController
from simulation.plants.dc_motor import DCMotor
from simulation.simulator import Simulator
from simulation.simulation_plotter import SimulationPlotter

def run_dcMotor_adrc_sim(setpoint=500, t_final=1):
    controller_dt = 0.0005
    motor = DCMotor()
    motor.set_motor_parameters(J=7.0865e-4, b=5.4177e-6, K=0.0061, R=0.0045, L=1.572e-4)

    # For DC motor, use a second-order ADRC
    # Estimate b0: For a DC motor, b0 can be approximated as 1/(J*L) for speed control
    J = 7.0865e-4
    L = 1.572e-4
    b0 = 1.0 / (J * L)

    controller = ADRCController(
        order=2,
        dt=controller_dt,
        bandwidth_eso=180.0,         
        bandwidth_controller=40.0,  
        b0=b0
    )
    controller.set_output_limits(-5.0, 5.0)

    setpoint_func = lambda t: setpoint
    sim = Simulator(motor, controller, setpoint_func, dt=controller_dt, t_final=t_final)
    results = sim.run()

    plotter = SimulationPlotter(results)
    plotter.plot_current_and_voltage()

    print("Final error:", abs(results["output"][-1] - setpoint))

if __name__ == "__main__":
    run_dcMotor_adrc_sim()
