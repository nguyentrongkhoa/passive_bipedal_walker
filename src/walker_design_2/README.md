# **What's new?**
- The choice of Bullet as the preferred physics engine in place of the default ODE.
- Augmentation of the URDF file with the Jinja templating engine so that a large number of desired simulation parameters (masses, mass ratios, joint angles, etc.) can be tested in a node.
# **Problems**
- As no kneelock mechanism is implemented in the simulation, the walker in most cases folds immediately after release due to the laxed knees.
