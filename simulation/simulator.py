
import numpy as np
from scipy.integrate import solve_ivp

class Simulator:
    def __init__(self, plant, controller, setpoint_func, dt=0.001, t_final=0.1):
        self.plant = plant
        self.controller = controller
        self.setpoint_func = setpoint_func
        self.dt = dt
        self.t_final = t_final
        self.x0 = np.array([0.0, 0.0])  # initial state [omega, current]

    def run(self):
        x = self.x0.copy()
        t_log, y_log, u_log, sp_log = [], [], [], []

        t_values = np.arange(0, self.t_final, self.dt)

        for t in t_values:
            y = self.plant.output(x)
            setpoint = self.setpoint_func(t)
            u = self.controller.update(setpoint, y)

            sol = solve_ivp(lambda t_, x_: self.plant.ode(t_, x_, u),
                            [t, t + self.dt], x,
                            method='RK45',
                            max_step=self.dt / 10,
                            t_eval=np.linspace(t, t + self.dt, 10))

            for t_i, x_i in zip(sol.t, sol.y.T):
                t_log.append(t_i)
                y_log.append(self.plant.output(x_i))
                u_log.append(u)
                sp_log.append(self.setpoint_func(t_i))

            x = sol.y[:, -1]

        return {
            "time": np.array(t_log),
            "setpoint": np.array(sp_log),
            "output": np.array(y_log),
            "manipulated variable": np.array(u_log),
        }
