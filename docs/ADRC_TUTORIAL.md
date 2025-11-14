# ADRC Control: Theory, Implementation & Tuning

This tutorial walks through the theory, mathematical derivation, and implementation of an **ADRC (Active Disturbance Rejection Control) controller** as used in this project for RL circuits and DC motors.

---

## 1. What is ADRC?

ADRC is a modern, robust control strategy that estimates and compensates for both internal dynamics and external disturbances in real time. Unlike PID, ADRC does not require an accurate model of the plant and can handle unknown disturbances and parameter variations.

**Key features:**
- Real-time disturbance estimation and rejection
- Model-independent (requires only an estimate of the input gain $b_0$)
- Fast response and robustness to uncertainties

---

## 2. ADRC Structure Overview

ADRC consists of two main parts:
- **Extended State Observer (ESO):** Estimates the system state and external disturbance
- **State Feedback Controller:** Uses the ESO estimates to compute the control signal

The general ADRC control law is:
$$
\nu(t) = u_0(t) - \hat{f}(t)$$
$$u(t) = \frac{\nu(t)}{b_0}$$
where $u_0(t)$ is the state feedback term and $\hat{f}(t)$ is the estimated total disturbance.

---

## 3. Mathematical Derivation

### 3.1 First-Order ADRC (for RL Circuit)

#### **Plant Model**
The RL circuit can be modeled as:
$$
\dot{i}(t) = -\frac{R}{L}i(t) + \frac{1}{L}u(t)
$$
or, in ADRC form:
$$
\dot{y}(t) = b_0 u(t) + f(t)
$$
where:
- $y(t)$: output (current)
- $b_0 = 1/L$
- $f(t)$: total disturbance (includes $-\frac{R}{L}i(t)$ and any external disturbance)

#### **Extended State Observer (ESO)**
We augment the state to estimate both $y$ and $f$:
$$
\begin{align*}
\dot{z}_1 &= z_2 + b_0 u - l_1(z_1 - y) \\
\dot{z}_2 &= -l_2(z_1 - y)
\end{align*}
$$
where:
- $z_1$: estimate of $y$
- $z_2$: estimate of $f$
- $l_1, l_2$: observer gains (set by observer bandwidth $\omega_o$)


#### **Control Law**
$$
u_0(t) = K_p(r - z_1)
$$
$$
\nu(t) = u_0(t) - z_2
$$
$$
u(t) = \frac{\nu(t)}{b_0}
$$
where:
- $u_0(t)$ is the state feedback term (proportional to the tracking error)
- $z_2$ is the estimated total disturbance from the ESO
- $\nu(t)$ is the virtual control input
- $u(t)$ is the actual control signal applied to the plant
- $K_p$ is the controller gain (set by controller bandwidth $\omega_c$)

#### **Tuning**
- **Observer gains:**
  - $l_1 = 2\omega_o$
  - $l_2 = \omega_o^2$
- **Controller gain:**
  - $K_p = \omega_c^2$ (aggressive), or $K_p = \omega_c$ (conservative)
- **$b_0$:** $1/L$ for RL circuit

---

### 3.2 Second-Order ADRC (for DC Motor)

#### **Plant Model**
A DC motor (speed control) can be approximated as a second-order system:
$$
\ddot{y}(t) = b_0 u(t) + f(t)
$$
where $y(t)$ is the speed, $b_0$ is an estimated input gain (e.g., $1/(J L)$), and $f(t)$ is the total disturbance.

#### **Extended State Observer (ESO)**
The ESO estimates output, its derivative, and the total disturbance:
$$
\begin{align*}
\dot{z}_1 &= z_2 - l_1(z_1 - y) \\
\dot{z}_2 &= z_3 + b_0 u - l_2(z_1 - y) \\
\dot{z}_3 &= -l_3(z_1 - y)
\end{align*}
$$
where:
- $z_1$: estimate of $y$
- $z_2$: estimate of $\dot{y}$
- $z_3$: estimate of $f$
- $l_1, l_2, l_3$: observer gains (set by observer bandwidth $\omega_o$)

#### **Control Law**
$$
u_0(t) = K_p (r - z_1) - K_d z_2
$$
$$
\nu(t) = u_0(t) - z_3
$$
$$
u(t) = \frac{\nu(t)}{b_0}
$$
where:
- $u_0(t)$ is the state feedback term (proportional and derivative action on the tracking error)
- $z_3$ is the estimated total disturbance from the ESO
- $\nu(t)$ is the virtual control input
- $u(t)$ is the actual control signal applied to the plant
- $K_p, K_d$ are controller gains (set by controller bandwidth $\omega_c$)

#### **Tuning**
- **Observer gains:**
  - $l_1 = 3\omega_o$
  - $l_2 = 3\omega_o^2$
  - $l_3 = \omega_o^3$
- **Controller gains:**
  - $K_p = \omega_c^2$
  - $K_d = 2\omega_c$
- **$b_0$:** $1/(J L)$ for DC motor

---

## 4. Practical Implementation in This Project

- **Python implementation:** See `adrc/adrcController.py`
- **First-order ADRC:** Used for RL circuit (`demos/adrc_RL_circuit.py`)
- **Second-order ADRC:** Used for DC motor (`demos/adrc_dc_motor.py`)
- **Tuning parameters:**
  - `dt`: sample time
  - `bandwidth_eso`: observer bandwidth $\omega_o$
  - `bandwidth_controller`: controller bandwidth $\omega_c$
  - `b0`: input gain estimate
- **Output limits:** Set to match actuator constraints

---

## 5. Tuning Guidelines

### Sample Time ($dt$)
- **Rule of thumb:** The sample time should be at least 10–20× faster than the dominant plant time constant ($\tau$).
- **Why:** Too slow sample time causes poor disturbance rejection and instability.

### Observer Bandwidth ($\omega_o$)
- **Guideline:** Set $\omega_o$ to be 3–10× higher than the controller bandwidth ($\omega_c$).
  - $\omega_o = 3\omega_c$ (conservative), $\omega_o = 10\omega_c$ (aggressive)
- **Constraint:** $\omega_o \cdot dt < 0.1$ (to avoid observer noise amplification)
- **Interpretation:** Higher $\omega_o$ means faster disturbance estimation, but more sensitivity to noise.

### Controller Bandwidth ($\omega_c$)
- **Guideline:** Set $\omega_c$ to be 3–5× lower than the inverse of the plant time constant.
  - $\omega_c = 1/(3\tau)$ to $1/(5\tau)$
- **Constraint:** $\omega_c \cdot dt < 0.2$ (to avoid instability)
- **Interpretation:** Higher $\omega_c$ means faster response, but more sensitivity to noise and model errors.

### Input Gain ($b_0$)
- Use plant knowledge:
  - RL circuit: $b_0 = 1/L$
  - DC motor: $b_0 = 1/(J L)$
- If uncertain, start with a conservative (lower) value and tune up as needed.


### Practical Troubleshooting
- **Overshoot/oscillation:** Lower $\omega_c$ and/or $\omega_o$
- **Slow response:** Increase $\omega_c$ (if $\omega_c \cdot dt < 0.2$)
- **Poor disturbance rejection:** Increase $\omega_o$ (if $\omega_o \cdot dt < 0.1$)
- **Noisy output:** Lower $\omega_o$ or add output filtering
- **Steady-state error:** Check $b_0$ estimate and disturbance observer performance

---

## 6. Application Examples

### RL Circuit (First-Order ADRC)
- See `demos/adrc_RL_circuit.py`
- Tuning in demo:
  - `dt = 0.0005`
  - `bandwidth_eso = 180`
  - `bandwidth_controller = 20`
  - `b0 = 1/L`

### DC Motor (Second-Order ADRC)
- See `demos/adrc_dc_motor.py`
- Tuning in demo:
  - `dt = 0.0005`
  - `bandwidth_eso = 180`
  - `bandwidth_controller = 40`
  - `b0 = 1/(J*L)`

---

## 7. Further Reading

- [ADRC Wikipedia](https://en.wikipedia.org/wiki/Active_disturbance_rejection_control)
- [Back to project overview](../README.md)
