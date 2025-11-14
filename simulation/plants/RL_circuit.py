class RLCircuit:
    """
    Simple RL circuit model: V = R*i + L*di/dt
    """

    def __init__(self, R=1.0, L=1.0):
        """
        Initialize the RL circuit with resistance R (Ohm) and inductance L (H).
        """
        self.R = R
        self.L = L
        self.current = 0.0  # Initial current (A)
        self.input_voltage = 0.0  # Applied voltage (V)

    def set_parameters(self, R, L):
        """
        Set the resistance and inductance.
        """
        self.R = R
        self.L = L

    def reset_state(self):
        """
        Reset the current and input voltage.
        """
        self.current = 0.0
        self.input_voltage = 0.0

    def ode(self, t, x, u):
        """
        Returns dx/dt for state x and input u.
        """
        i = x[0]
        di_dt = (u - self.R * i) / self.L
        return [di_dt]
    
    def output(self, x):
        """
        Get the current (A).
        """
        return x[0]
