import sys
from pathlib import Path

project_root = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(project_root))


from adrc.adrcController import ADRCController
from simulation.plants.RL_circuit import RLCircuit
from simulation.simulator import Simulator
from simulation.simulation_plotter import SimulationPlotter


def run_rl_circuit_magnitude_optimum(R=1.0, L=0.05, setpoint=0.1, t_final=0.05):
    controller_dt = 0.0005
    plant = RLCircuit(R, L)

    # RL circuit is a first-order system: di/dt = (V - R*i)/L
    # So b0 = 1/L for voltage input to current output
    b0 = 1.0 / L
    
    # Much more conservative bandwidths
    # ESO bandwidth should be moderate to avoid noise amplification
    # Controller bandwidth should give reasonable response time
    controller = ADRCController(order=1, dt=controller_dt, 
                              bandwidth_eso=180, bandwidth_controller=20, b0=b0)

    controller.set_output_limits(-5.0, 5.0)

    setpoint_func = lambda t: setpoint
    sim = Simulator(plant, controller, setpoint_func, dt=controller_dt, t_final=t_final)
    results = sim.run()

    plotter = SimulationPlotter(results)
    plotter.plot_current_and_voltage()

    # Optional: print final error
    print("Final error:", abs(results["output"][-1] - setpoint))


if __name__ == "__main__":
    run_rl_circuit_magnitude_optimum()
