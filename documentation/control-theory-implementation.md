# Control Theory Implementation Details

## Overview
This document provides detailed technical information about the control algorithms implemented in this robotics project. Each control method was carefully designed and tuned for the CRS Catalyst-5 robot arm.

## 1. PID Control with Anti-Windup

### Theory
The discrete-time PID controller implements the standard form with integral anti-windup to prevent actuator saturation issues.

### Implementation Details
```c
// Sampling time: 5ms (200Hz)
#define Ts 0.005

// PID gains (tuned experimentally)
float Kp[3] = {350.0, 300.0, 200.0};  // Joint 1, 2, 3
float Ki[3] = {40.0, 35.0, 25.0};
float Kd[3] = {15.0, 12.0, 8.0};

// Anti-windup limits
float integral_limit = 50.0;  // Prevents integral term explosion
```

### Tuning Process
1. Started with Ziegler-Nichols method for initial gains
2. Fine-tuned using step response analysis
3. Reduced derivative gain to minimize noise amplification
4. Added low-pass filter on derivative term (fc = 20Hz)

## 2. Feedforward Control

### Concept
Feedforward improves tracking by compensating for known system dynamics before errors occur.

### Components
- **Velocity Feedforward:** Compensates for back-EMF
- **Acceleration Feedforward:** Compensates for inertia
- **Gravity Compensation:** Static torque based on joint angles

### Equations
```
τ_ff = M(q)·q̈_desired + C(q,q̇)·q̇ + G(q)

Where:
- M(q): Inertia matrix
- C(q,q̇): Coriolis/centrifugal terms
- G(q): Gravity vector
```

## 3. Impedance Control

### Objective
Make the robot behave like a mass-spring-damper system for safe interaction.

### Control Law
```
F = K(x_d - x) + B(ẋ_d - ẋ) + M(ẍ_d - ẍ)
```

### Variable Impedance
- Stiffness K: 10-1000 N/m (task-dependent)
- Damping B: Critically damped (B = 2√(KM))
- Virtual mass M: 1-10 kg

### Application
Used in the egg manipulation task where compliance was critical to avoid breaking the egg.

## 4. Adaptive Friction Compensation

### Friction Model
Combined Coulomb and viscous friction:
```
F_friction = Fc·sign(v) + Fv·v
```

### Adaptation Algorithm
Gradient descent with momentum:
```c
// Learning rates
float alpha_coulomb = 0.01;
float alpha_viscous = 0.005;
float momentum = 0.9;

// Update rules
Fc += alpha_coulomb * e * sign(v) + momentum * prev_Fc_update;
Fv += alpha_viscous * e * v + momentum * prev_Fv_update;
```

### Convergence
- Typical convergence time: 2-3 seconds
- Steady-state error reduction: 80-90%

## 5. Trajectory Generation

### Cubic Spline Method
Ensures smooth position, velocity, and acceleration profiles.

### Constraints
- Velocity limit: 180°/s
- Acceleration limit: 360°/s²
- Jerk minimization through 5th-order blending

### Path Planning Algorithm
1. Waypoint specification in joint space
2. Time allocation based on distance and constraints
3. Cubic polynomial fitting between waypoints
4. Velocity/acceleration continuity enforcement

## 6. Multi-Modal Control

### Mode Switching
Smooth transitions between control modes using blending:
```c
output = α·mode1_output + (1-α)·mode2_output
// α transitions from 0 to 1 over 0.5 seconds
```

### Available Modes
1. **Position Control:** High stiffness, precise positioning
2. **Velocity Control:** Smooth motion, medium stiffness
3. **Force Control:** Low stiffness, compliant behavior
4. **Hybrid Position/Force:** Different modes per axis

## 7. Safety Features

### Joint Limits
- Soft limits: Gradual deceleration near boundaries
- Hard limits: Emergency stop if exceeded

### Velocity Limiting
```c
if (fabs(velocity) > max_velocity) {
    command = prev_command + sign(velocity) * max_decel * Ts;
}
```

### Fault Detection
- Encoder fault detection (missed counts)
- Current limit monitoring
- Communication timeout (100ms)

## Performance Validation

### Step Response Metrics
- Rise time: 0.3-0.5s (depending on step size)
- Overshoot: < 5%
- Steady-state error: < 0.1°

### Tracking Performance
- RMS error during trajectory: < 1°
- Maximum error: < 2° (at high accelerations)

### Computational Efficiency
- Control loop execution: 2.8ms (56% CPU utilization)
- Worst-case execution: 4.2ms (84% CPU utilization)

## Lessons Learned

1. **Friction Compensation:** Critical for low-velocity performance
2. **Feedforward:** Reduces lag in trajectory tracking by 60-70%
3. **Adaptive Control:** Essential for handling varying payloads
4. **Real-Time Constraints:** Required careful optimization of matrix operations

## References
- Spong, Hutchinson, Vidyasagar - "Robot Modeling and Control"
- Siciliano, Khatib - "Springer Handbook of Robotics"
- Craig - "Introduction to Robotics: Mechanics and Control"