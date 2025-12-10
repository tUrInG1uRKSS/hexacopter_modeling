# Hexacopter Dynamics Modeling and Simulation

## Overview

This project provides a complete MATLAB-based simulation environment for the dynamics and control of a Hexacopter UAV. The primary goal of this work is to model the rigid body dynamics of the drone using two distinct mathematical approaches for attitude representation:

1.  **Euler Angles (ZYX Convention):** Intuitive but susceptible to mathematical singularities (Gimbal Lock).
2.  **Unit Quaternions:** A robust, 4-dimensional representation that avoids singularities and allows for complex acrobatic maneuvers.

The simulation includes a 3D visualization engine, automated flight scenarios, and a comparative analysis of the two kinematic models.

## 📂 Repository Structure

The project is organized as follows:

```text
├── code/
│   ├── test_drone.m             # MAIN SCRIPT: Configure and run simulations
│   ├── generate_test_case.m     # Defines initial conditions and thrust commands for scenarios
│   ├── dynamics_model_euler.m   # Physics engine using Euler Angle kinematics (12 states)
│   ├── dynamics_model_quat.m    # Physics engine using Quaternion kinematics (13 states)
│   ├── animations.m             # 3D Visualization engine
│   ├── plot_all.m               # Generates 2D analysis plots (Position, Velocity, Rates)
│   └── utils/                   # Helper functions (rotx, roty, rotz, quatToRotm)
├── images/                      # Simulation results and diagrams
├── sources/                     # Reference literature and documentation
└── Report_Drone.pdf             # Detailed project report
```

## 🚀 Getting Started

### Prerequisites

  * **MATLAB** (The code uses standard functions and the `ode45` solver).

### Installation & Usage

1.  Clone this repository:
    ```bash
    git clone https://github.com/your-username/hexacopter_modeling.git
    ```
2.  Open MATLAB and navigate to the `code/` directory.
3.  Open the main script **`test_drone.m`**.
4.  Configure the simulation parameters at the top of the file:
    ```matlab
    test_case   = 1;         % Select scenario (0 to 6)
    model_type  = 'euler';   % Choose 'euler' or 'quat'
    animate_flag = true;     % Enable 3D animation
    ```
5.  Run the script. The simulation will calculate the trajectory, generate plots, and launch the 3D animation.

## 🧪 Simulation Scenarios (Test Cases)

The `generate_test_case.m` file defines 7 specific scenarios to validate the physics engine:

| Case | Description | Objective |
| :--- | :--- | :--- |
| **0** | **Hover Test** | Verify equilibrium forces. The drone should remain perfectly still at (0,0,0). |
| **1** | **Yaw Rotation** | Verify torque generation ($M_z$). Drone rotates around Z-axis without changing altitude. |
| **2** | **Vertical Takeoff** | Verify thrust ($T > mg$). Drone accelerates upward. |
| **3** | **Pitch Forward** | Verify torque ($M_y$) and translation coupling. Drone tilts nose-up/down and accelerates in X. |
| **4** | **Roll Left** | Verify torque ($M_x$) and translation coupling. Drone tilts sideways and accelerates in Y. |
| **5** | **Gimbal Lock** | **Critical Test.** Initializes pitch at $\theta = 90^\circ$. <br>• **Euler:** Fails/Unstable due to singularity. <br>• **Quaternion:** Handles dynamics correctly. |
| **6** | **Combined** | Takeoff + Yaw rotation. Validates coupled dynamics. |

## 📐 Mathematical Modeling

### Euler Formulation

  * **State Vector (12x1):** Position ($x,y,z$), Velocity ($v_x, v_y, v_z$), Angular Rates ($p,q,r$), Euler Angles ($\phi, \theta, \psi$).
  * **Limitation:** The kinematic matrix $W^{-1}$ contains a $\tan(\theta)$ term, which becomes undefined when the pitch angle $\theta = \pm 90^\circ$.

### Quaternion Formulation

  * **State Vector (13x1):** Position, Velocity, Angular Rates, Quaternion ($q_0, q_1, q_2, q_3$).
  * **Advantage:** The kinematic equation $\dot{\mathbf{q}} = \frac{1}{2} \Omega(\omega) \mathbf{q}$ is linear and singularity-free, making it suitable for all flight envelopes.

## 📊 Visualization

The project includes a custom 3D animation script (`animations.m`) that renders the drone's body, arms, and rotors in real-time based on the simulation data. It automatically adapts to visualize attitude data from either Euler angles or Quaternions.

## 📚 References

This modeling work is based on standard Newton-Euler equations for rigid bodies. Key references included in the `sources/` folder:

  * *Modelling and Controls of a Hexacopter*, Kuldeep Singh (Thesis, 2018).
  * Additional papers on UAV dynamics and Quaternion kinematics.

-----

*This project was developed for educational purposes to demonstrate the importance of attitude parametrization in flight dynamics.*
