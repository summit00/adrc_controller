
import matplotlib.pyplot as plt

class SimulationPlotter:
    def __init__(self, results):
        self.t = results["time"]
        self.sp = results["setpoint"]
        self.y = results["output"]
        self.u = results["manipulated variable"]

    def plot_current_and_voltage(self):
        plt.figure(figsize=(10, 5))
        plt.subplot(2, 1, 1)
        plt.plot(self.t, self.sp, '--', label='Setpoint (A)')
        plt.step(self.t, self.y, label='Measured Current (A)')
        plt.ylabel("Current")
        plt.legend()
        plt.grid()

        plt.subplot(2, 1, 2)
        plt.step(self.t, self.u, label='Applied Voltage (V)', color='tab:red')
        plt.xlabel("Time (s)")
        plt.ylabel("Voltage")
        plt.legend()
        plt.grid()

        plt.tight_layout()
        plt.show()
