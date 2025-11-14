import numpy as np


class ADRCController:
    """
    Active Disturbance Rejection Control (ADRC) controller implementation.
    
    ADRC is an advanced control technique that estimates and compensates for both
    internal dynamics and external disturbances using an Extended State Observer (ESO).
    It consists of:
        - Extended State Observer (ESO): estimates states and total disturbance
        - State feedback controller: generates control signal based on estimated states
    
    This implementation supports both first-order and second-order ADRC controllers.
    """

    def __init__(self, order: int, dt: float, bandwidth_eso: float, bandwidth_controller: float, b0: float = 1.0):
        """
        Initialize the ADRC controller with automatic tuning based on bandwidths.
        
        Args:
            order: Order of the system (1 for first-order, 2 for second-order)
            dt: Sampling time (seconds)
            bandwidth_eso: ESO bandwidth (determines observer convergence speed)
            bandwidth_controller: Controller bandwidth (determines control aggressiveness)
            b0: Control effectiveness parameter (plant input gain estimate)
        """
        if order not in [1, 2]:
            raise ValueError("Order must be 1 or 2")
        
        # Validate sample time vs bandwidth for numerical stability
        self._validate_bandwidth_vs_sample_time(dt, bandwidth_eso, bandwidth_controller)
        
        self.order = order
        self.dt = dt
        self.bandwidth_eso = bandwidth_eso
        self.bandwidth_controller = bandwidth_controller
        self.b0 = b0
        
        # Automatic tuning based on bandwidths
        self._tune_controller()

        # Initialize ESO states
        if self.order == 1:
            # First-order: [z1, z2] = [output, disturbance]
            self.z = np.array([0.0, 0.0])
        else:
            # Second-order: [z1, z2, z3] = [output, derivative, disturbance]
            self.z = np.array([0.0, 0.0, 0.0])

        # Output limits (default: very large range)
        self.out_min = -1e9
        self.out_max = 1e9
        
        # Flag to initialize ESO with first measurement
        self._first_update = True

    def _validate_bandwidth_vs_sample_time(self, dt: float, bandwidth_eso: float, bandwidth_controller: float):
        """
        Validate that the bandwidths are not too high for the given sample time.
        High bandwidths with large sample times can cause numerical instability.
        
        Args:
            dt: Sample time (seconds)
            bandwidth_eso: ESO bandwidth
            bandwidth_controller: Controller bandwidth
            
        Raises:
            AssertionError: If bandwidth is too high for the sample time
        """
        # Rule of thumb: bandwidth * dt should be much less than 1 for stability
        # Conservative limit: bandwidth * dt < 0.1 (Nyquist-like criterion)
        max_bandwidth_eso_ratio = 0.1
        max_bandwidth_controller_ratio = 0.2  # Controller can be slightly more aggressive
        
        eso_ratio = bandwidth_eso * dt
        controller_ratio = bandwidth_controller * dt
        
        assert eso_ratio < max_bandwidth_eso_ratio, (
            f"ESO bandwidth too high for sample time! "
            f"ESO bandwidth * dt = {eso_ratio:.4f} >= {max_bandwidth_eso_ratio}. "
            f"Reduce ESO bandwidth to < {max_bandwidth_eso_ratio / dt:.1f} "
            f"or decrease sample time to < {max_bandwidth_eso_ratio / bandwidth_eso:.6f} seconds."
        )
        
        assert controller_ratio < max_bandwidth_controller_ratio, (
            f"Controller bandwidth too high for sample time! "
            f"Controller bandwidth * dt = {controller_ratio:.4f} >= {max_bandwidth_controller_ratio}. "
            f"Reduce controller bandwidth to < {max_bandwidth_controller_ratio / dt:.1f} "
            f"or decrease sample time to < {max_bandwidth_controller_ratio / bandwidth_controller:.6f} seconds."
        )
        
        # Additional check: ESO bandwidth should typically be higher than controller bandwidth
        if bandwidth_eso <= bandwidth_controller:
            print(f"Warning: ESO bandwidth ({bandwidth_eso}) should typically be 3-10 times "
                  f"higher than controller bandwidth ({bandwidth_controller}) for good performance.")

    def _tune_controller(self):
        """
        Automatic tuning of controller and ESO parameters based on bandwidths.
        Following MATLAB ADRC structure:
        - Controller pole: (s + ωc) → Kp = ωc
        - Observer poles: (s + ωo)² → l1 = 2ωo, l2 = ωo²
        """
        wo = self.bandwidth_eso
        wc = self.bandwidth_controller
        
        if self.order == 1:
            # First-order ESO gains for observer poles at (s + ωo)²
            self.l1 = 2 * wo
            self.l2 = wo * wo
            
            # First-order controller gain for desired bandwidth ωc
            # For first-order system, to achieve bandwidth ωc, we need higher gain
            # Based on closed-loop analysis: Kp should be much higher than ωc
            self.kp = wc * wc  # More aggressive tuning for faster response
            self.kd = 0.0
            
        else:
            # Second-order ESO gains (to be implemented later)
            self.l1 = 3 * wo
            self.l2 = 3 * wo * wo
            self.l3 = wo * wo * wo
            
            # Second-order controller gains (to be implemented later)
            self.kp = wc * wc
            self.kd = 2 * wc

    def set_output_limits(self, out_min: float, out_max: float):
        """
        Set the minimum and maximum output limits to prevent excessive control signal.
        
        Args:
            out_min: Minimum output value
            out_max: Maximum output value
        """
        self.out_min = out_min
        self.out_max = out_max

    def reset(self):
        """
        Reset the controller state (ESO states).
        """
        if self.order == 1:
            self.z = np.array([0.0, 0.0])
        else:
            self.z = np.array([0.0, 0.0, 0.0])
        self._first_update = True

    def _update_eso_first_order(self, measurement: float, control_input: float):
        """
        Update the Extended State Observer for first-order system.
        
        Args:
            measurement: Current measured output
            control_input: Applied control input
        """
        # ESO error
        e1 = self.z[0] - measurement
        
        # ESO dynamics for first-order system
        # z1_dot = z2 + b0*u - l1*e1
        # z2_dot = -l2*e1
        z1_dot = self.z[1] + self.b0 * control_input - self.l1 * e1
        z2_dot = -self.l2 * e1
        
        # Euler integration
        self.z[0] += z1_dot * self.dt
        self.z[1] += z2_dot * self.dt

    def _update_eso_second_order(self, measurement: float, control_input: float):
        """
        Update the Extended State Observer for second-order system.
        
        Args:
            measurement: Current measured output
            control_input: Applied control input
        """
        # ESO error
        e1 = self.z[0] - measurement
        
        # ESO dynamics for second-order system
        # z1_dot = z2 - l1*e1
        # z2_dot = z3 + b0*u - l2*e1
        # z3_dot = -l3*e1
        z1_dot = self.z[1] - self.l1 * e1
        z2_dot = self.z[2] + self.b0 * control_input - self.l2 * e1
        z3_dot = -self.l3 * e1
        
        # Euler integration
        self.z[0] += z1_dot * self.dt
        self.z[1] += z2_dot * self.dt
        self.z[2] += z3_dot * self.dt

    def update(self, setpoint: float, measurement: float) -> float:
        """
        Compute the ADRC control output based on the current setpoint and measurement.
        
        Args:
            setpoint: Desired target value
            measurement: Current measured value
        
        Returns:
            Control output (clamped within set limits)
        """
        # Initialize ESO with first measurement
        if self._first_update:
            self.z[0] = measurement
            self._first_update = False
        
        # Generate control signal based on current estimates
        # Following MATLAB implementation: u(t) = (u0(t) - z2) / b0
        if self.order == 1:
            # First-order ADRC: u = (kp*(r-z1) - z2) / b0
            # where u0 = kp*(r-z1), r is setpoint, z1 is estimated output, z2 is estimated disturbance
            u0 = self.kp * (setpoint - self.z[0])
            output = (u0 - self.z[1]) / self.b0
        else:
            # Second-order ADRC: u = (kp*(r-z1) - kd*z2 - z3) / b0
            # where z2 is estimated derivative, z3 is estimated disturbance
            u0 = self.kp * (setpoint - self.z[0]) - self.kd * self.z[1]
            output = (u0 - self.z[2]) / self.b0
        
        # Clamp the output within specified limits
        output = max(min(output, self.out_max), self.out_min)
        
        # Update ESO with current measurement and control input
        if self.order == 1:
            self._update_eso_first_order(measurement, output)
        else:
            self._update_eso_second_order(measurement, output)

        return output

    def get_estimated_states(self) -> np.ndarray:
        """
        Get the current estimated states from the ESO.
        
        Returns:
            Array of estimated states:
            - First-order: [output, disturbance]
            - Second-order: [output, derivative, disturbance]
        """
        return self.z.copy()

    def get_estimated_disturbance(self) -> float:
        """
        Get the current estimated total disturbance.
        
        Returns:
            Estimated total disturbance value
        """
        return self.z[-1]  # Last state is always the disturbance estimate
