# Controllers folder
This folder contains various Simulink models together with their inizialization files (.m)

## The simulink models
- [test.slx](test.slx) - 
This model tests the possibility of running a part of the model while the other waits for certain intants
- [SpringMPCmimo.slx](SpringMPCmimo.slx) - 
Model containing an MPC controller for a mass-damper-spring system
- [SpringiLQR.slx](SpringiLQR.slx) - 
Implementation of iterative LQR compared with a PID for a mass-damper-spring system
- [SimpleMPC.slx](SimpleMPC.slx) - 
Attempt to design an MPC for a simple vehicle model (non linear)
- [SimpleLQR.slx](SimpleLQR.slx) - 
Attempt to design an LQR for a simple vehicle model (non linear)
- [MRACDevastator.slx](MRACDevastator.slx) - 
Simple MRAC applied to a simple second order system. To be used for Devastator
- [MPC.slx](MPC.slx) - 
Attempt at usign MPC for Desavtator current model
- [LQR.slx](LQR.slx) - 
Attempt at usign LQR for Desavtator current model
- [LinearMPCsiso.slx](LinearMPCsiso.slx) - 
MPC implementation for a linear single-input single-output system
- [LinearMPCmimo.slx](LinearMPCmimo.slx) - 
MPC implementation for a linear multiple-input multiple-output system
- [LinearMPC.slx](LinearMPC.slx) - 
MPC implementation for a linear single-input single-output system. For equal results run [reced.m](reced.m)

## The MATLAB scripts
- [underOsc.m](underOsc.m) - 
Example 2.1 : MPC implementation for an Undamped Oscillator from this [book](https://www.google.com/url?sa=t&rct=j&q=&esrc=s&source=web&cd=&ved=2ahUKEwjs-N6VhNvzAhVR6aQKHUMfByIQFnoECAUQAQ&url=https%3A%2F%2Fwww.researchgate.net%2Fprofile%2FMohamed-Mourad-Lafifi%2Fpost%2FWhat_are_the_possible_combination_of_controllers_exists_with_MPCModel_predictive_control%2Fattachment%2F604610505d920200013be2f8%2FAS%253A999053953363973%25401615204432144%2Fdownload%2FModel%2BPredictive%2BContro%2BlSystem%2BDesign%2Band%2BImplementation%2BUsing%2BMATLAB_Wang.pdf&usg=AOvVaw1Xy3A6Az0r0y00ueUBo76M)
- [SpringiLQR.m](SpringiLQR.m) - 
Initialization for LQR matrices and other values for SpringiLQR.slx
- [simpleLQR.m](simpleLQR.m) - 
Initialization for LQR matrices and other values for SimpleLQR.slx and 
- [reced.m](reced.m) - 
Tutorial 1.3 : Implementation of Receding Horizon Control from this [book](https://www.google.com/url?sa=t&rct=j&q=&esrc=s&source=web&cd=&ved=2ahUKEwjs-N6VhNvzAhVR6aQKHUMfByIQFnoECAUQAQ&url=https%3A%2F%2Fwww.researchgate.net%2Fprofile%2FMohamed-Mourad-Lafifi%2Fpost%2FWhat_are_the_possible_combination_of_controllers_exists_with_MPCModel_predictive_control%2Fattachment%2F604610505d920200013be2f8%2FAS%253A999053953363973%25401615204432144%2Fdownload%2FModel%2BPredictive%2BContro%2BlSystem%2BDesign%2Band%2BImplementation%2BUsing%2BMATLAB_Wang.pdf&usg=AOvVaw1Xy3A6Az0r0y00ueUBo76M)
- [main.m](main.m) - 
First attempt at MPC implementation for MPC.slx
- [LQR.m](LQR.m) - 
First attempt at LQR implementation for LQR.slx
- [extmodel.m](extmodel.m) - 
Some examples from the [book](https://www.google.com/url?sa=t&rct=j&q=&esrc=s&source=web&cd=&ved=2ahUKEwjs-N6VhNvzAhVR6aQKHUMfByIQFnoECAUQAQ&url=https%3A%2F%2Fwww.researchgate.net%2Fprofile%2FMohamed-Mourad-Lafifi%2Fpost%2FWhat_are_the_possible_combination_of_controllers_exists_with_MPCModel_predictive_control%2Fattachment%2F604610505d920200013be2f8%2FAS%253A999053953363973%25401615204432144%2Fdownload%2FModel%2BPredictive%2BContro%2BlSystem%2BDesign%2Band%2BImplementation%2BUsing%2BMATLAB_Wang.pdf&usg=AOvVaw1Xy3A6Az0r0y00ueUBo76M)
- [APF_matlab.m](APF_matlab.m) - 
APF trajectory planner from Enza

## The MATLAB functions
- [QPhild.m](QPhild.m) - 
Tutorial 2.1: Hildreth's Quadratic Programming [book](https://www.google.com/url?sa=t&rct=j&q=&esrc=s&source=web&cd=&ved=2ahUKEwjs-N6VhNvzAhVR6aQKHUMfByIQFnoECAUQAQ&url=https%3A%2F%2Fwww.researchgate.net%2Fprofile%2FMohamed-Mourad-Lafifi%2Fpost%2FWhat_are_the_possible_combination_of_controllers_exists_with_MPCModel_predictive_control%2Fattachment%2F604610505d920200013be2f8%2FAS%253A999053953363973%25401615204432144%2Fdownload%2FModel%2BPredictive%2BContro%2BlSystem%2BDesign%2Band%2BImplementation%2BUsing%2BMATLAB_Wang.pdf&usg=AOvVaw1Xy3A6Az0r0y00ueUBo76M)
- [mpcgains.m](mpcgains.m) -
 Tutorial 1.2: Computation of MPC Gains from state-space model


## The MAT-Files
- Model_nlarx.mat - 
Model of the Devastator obtained from experimental data

## The folders
- Complete model: Contains the recent work about the real Devastator model
- Figures: Any debugging graphs or stuff I find interesting

## References for these models and codes

[1] [Abbeel, P. & UC Berkeley EECS. (n.d.). Optimal Control for Linear Dynamical Systems and Quadratic Cost (“LQR”).](https://people.eecs.berkeley.edu/~pabbeel/cs287-fa12/slides/LQR.pdf)
[2] [Huang, Y. (2020). Model Reduction and Iterative LQR for Control of High-Dimensional Nonlinear Systems.](https://escholarship.org/uc/item/39b4z63z)
[3] [Abbeel, P. & UC Berkeley EECS. (n.d.). Optimal Control for Linear Dynamical Systems and Quadratic Cost (“LQR”).](https://www.cds.caltech.edu/~murray/courses/cds110/wi06/lqr.pdf)
[4] [Model Predictive Control System Design and Implementation Using MATLAB® (Advances in Industrial Control) by Liuping Wang (2010–10-21). (2021). Springer; Softcover reprint of hardcover 1st ed. 2009 edition (2010–10-21).](https://www.google.com/url?sa=t&rct=j&q=&esrc=s&source=web&cd=&ved=2ahUKEwjs-N6VhNvzAhVR6aQKHUMfByIQFnoECAUQAQ&url=https%3A%2F%2Fwww.researchgate.net%2Fprofile%2FMohamed-Mourad-Lafifi%2Fpost%2FWhat_are_the_possible_combination_of_controllers_exists_with_MPCModel_predictive_control%2Fattachment%2F604610505d920200013be2f8%2FAS%253A999053953363973%25401615204432144%2Fdownload%2FModel%2BPredictive%2BContro%2BlSystem%2BDesign%2Band%2BImplementation%2BUsing%2BMATLAB_Wang.pdf&usg=AOvVaw1Xy3A6Az0r0y00ueUBo76M)
