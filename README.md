# Controllers folder
This folder contains various Simulink models together with their inizialization files (.m)

## The simulink models
- [test.slx](test.slx)
This model tests the possibility of running a part of the model while the other waits for certain intants
- [SpringMPCmimo.slx](SpringMPCmimo.slx)
Model containing an MPC controller for a mass-damper-spring system
- [SpringiLQR.slx](SpringiLQR.slx)
Implementation of iterative LQR compared with a PID for a mass-damper-spring system
- [SimpleMPC.slx](SimpleMPC.slx)
Attempt to design an MPC for a simple vehicle model (non linear)
- [SimpleLQR.slx](SimpleLQR.slx)
Attempt to design an LQR for a simple vehicle model (non linear)
- [MRACDevastator.slx](MRACDevastator.slx)
Simple MRAC applied to a simple second order system. To be used for Devastator
- [MPC.slx](MPC.slx)
Attempt at usign MPC for Desavtator current model
- [LQR.slx](LQR.slx)
Attempt at usign LQR for Desavtator current model
- [LinearMPCsiso.slx](LinearMPCsiso.slx)
MPC implementation for a linear single-input single-output system
- [LinearMPCmimo.slx](LinearMPCmimo.slx)
MPC implementation for a linear multiple-input multiple-output system
- [LinearMPC.slx](LinearMPC.slx)
MPC implementation for a linear single-input single-output system. For equal results run [reced.m](reced.m)

## The MATLAB codes






## References for these models and codes

- [Berkeley presentation](https://people.eecs.berkeley.edu/~pabbeel/cs287-fa12/slides/LQR.pdf)
- [Model Reduction and Iterative LQR for Control of High-Dimensional Nonlinear Systems](https://escholarship.org/uc/item/39b4z63z)
- [LQR Control- Caltech](https://www.cds.caltech.edu/~murray/courses/cds110/wi06/lqr.pdf)