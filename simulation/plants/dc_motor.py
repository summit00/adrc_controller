import math

class DCMotor:
    """
    Simple DC motor model based on the electrical and mechanical dynamics:
    
    Electrical:  V = R*i + L*di/dt + K*omega
    Mechanical:  J*dω/dt = K*i - b*ω
    
    Where:
        V     = Applied voltage (V)
        i     = Armature current (A)
        ω     = Angular velocity (rad/s)
        J     = Inertia (kg·m^2)
        b     = Damping coefficient (N·m·s)
        K     = Motor torque constant and back-EMF constant (N·m/A and V·s/rad)
        R     = Armature resistance (Ω)
        L     = Armature inductance (H)
    """

    def __init__(self):
        """
        Initialize the DC motor with default parameters (typical small DC motor).
        """
        # Motor state variables
        self.position = 0.0       # Rotor position θ (rad)
        self.velocity = 0.0       # Angular velocity ω (rad/s)
        self.current = 0.0        # Armature current i (A)
        self.input_voltage = 0.0  # Applied voltage V (V)

        # Motor parameters
        self.J = 0.01  # Moment of inertia (kg·m^2)
        self.b = 0.1   # Damping coefficient (N·m·s)
        self.K = 0.01  # Torque and back-EMF constant (N·m/A and V·s/rad)
        self.R = 1.0   # Armature resistance (Ω)
        self.L = 0.5   # Armature inductance (H)

    def set_motor_parameters(self, J, b, K, R, L):
        """
        Set motor physical parameters:
        J (kg·m^2), b (N·m·s), K (N·m/A), R (Ω), L (H).
        """
        self.J = J
        self.b = b
        self.K = K
        self.R = R
        self.L = L

    def reset_state(self):
        """
        Reset the motor state variables: position, velocity, current, and input voltage.
        """
        self.position = 0.0
        self.velocity = 0.0
        self.current = 0.0
        self.input_voltage = 0.0

    def ode(self, t, x, u):
        """
        Compute the derivatives of the state variables for the DC motor.

        State vector:
            x[0] = ω (angular velocity in rad/s)
            x[1] = i (current in A)
        Input:
            u = applied voltage (V)
        
        Returns:
            [dω/dt, di/dt]
        """
        omega, current = x

        # Electrical dynamics: di/dt = (u - R*i - K*ω) / L
        di_dt = (u - self.R * current - self.K * omega) / self.L

        # Mechanical dynamics: dω/dt = (K*i - b*ω) / J
        domega_dt = (self.K * current - self.b * omega) / self.J

        return [domega_dt, di_dt]
    
    def output(self, x):
        """
        Convert angular velocity (rad/s) to RPM.
        """
        return x[0] * 30 / math.pi  # RPM
