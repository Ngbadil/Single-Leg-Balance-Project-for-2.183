# Single-Leg-Balance-Project-for-2.183

For the time being, everyone will have their own branch they can commit changes to. Once changes are reviewed and tested we will merge branches to main.



This project builds on an existing experimental dataset collected from younger and older participants performing the One-Legged Stand Test (OLST), a widely used clinical assessment of balance, and extends prior work that conducted a frequency-dependent intersection point height (zIP) analysis. The goal of this project is to develop a double-inverted pendulum (DIP) model of human balance during OLST and integrate a Linear Quadratic Regulator (LQR) optimal control framework to better interpret observed zIP control patterns and their relationship to fall risk.

We will implement a sagittal-plane DIP model in MATLAB and apply an LQR optimal control framework to identify best-fit biomechanical and control parameters at both the population and individual participant levels. In particular, we aim to recover interpretable parameters such as joint torque weighting (β) and joint noise weighting (σ), which may help link zIP curves to underlying central nervous system control strategies and provide a more mechanistic interpretation of balance control differences.

We will also examine how variations in model parameters, such as center of mass (CoM) height and moment of inertia distribution, affect balance control, allowing us to simulate demographic differences (e.g., age-related changes) and task-specific characteristics. The framework will ultimately be extended to the frontal plane to capture mediolateral balance dynamics during OLST.

By combining experimental data, zIP analysis, and optimal control modeling, this project aims to establish a more neuro-biomechanically grounded and interpretable framework for understanding balance control and its relationship to fall risk.
