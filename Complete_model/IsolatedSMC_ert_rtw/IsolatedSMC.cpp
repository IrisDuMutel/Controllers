//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: IsolatedSMC.cpp
//
// Code generated for Simulink model 'IsolatedSMC'.
//
// Model version                  : 1.2
// Simulink Coder version         : 9.4 (R2020b) 29-Jul-2020
// C/C++ source code generated on : Thu Dec  2 11:41:18 2021
//
// Target selection: ert.tlc
// Embedded hardware selection: Intel->x86-64 (Windows64)
// Code generation objectives: Unspecified
// Validation result: Not run
//
#include "IsolatedSMC.h"
#include "IsolatedSMC_private.h"

// Model step function
void IsolatedSMCModelClass::step()
{
  real_T rtb_TSamp;
  real_T rtb_TSamp_h;
  real_T rtb_TSamp_o;
  real_T rtb_TSamp_p0;
  real_T rtb_errore;
  real_T rtb_errore_0;

  // SampleTimeMath: '<S1>/TSamp' incorporates:
  //   Inport: '<Root>/Vx_ref'
  //
  //  About '<S1>/TSamp':
  //   y = u * K where K = 1 / ( w * Ts )

  rtb_TSamp = IsolatedSMC_U.Vx_ref * 100.0;

  // SampleTimeMath: '<S2>/TSamp' incorporates:
  //   Inport: '<Root>/Vx'
  //
  //  About '<S2>/TSamp':
  //   y = u * K where K = 1 / ( w * Ts )

  rtb_TSamp_o = IsolatedSMC_U.Vx * 100.0;

  // MATLAB Function: '<Root>/MATLAB Function1' incorporates:
  //   Inport: '<Root>/Vx'
  //   Inport: '<Root>/Vx_ref'
  //   Sum: '<Root>/Sum'
  //   Sum: '<Root>/Sum1'
  //   Sum: '<S1>/Diff'
  //   Sum: '<S2>/Diff'
  //   UnitDelay: '<S1>/UD'
  //   UnitDelay: '<S2>/UD'
  //
  //  Block description for '<S1>/Diff':
  //
  //   Add in CPU
  //
  //  Block description for '<S2>/Diff':
  //
  //   Add in CPU
  //
  //  Block description for '<S1>/UD':
  //
  //   Store in Global RAM
  //
  //  Block description for '<S2>/UD':
  //
  //   Store in Global RAM

  rtb_errore = ((rtb_TSamp - IsolatedSMC_DW.UD_DSTATE) - (rtb_TSamp_o -
    IsolatedSMC_DW.UD_DSTATE_o)) + (IsolatedSMC_U.Vx_ref - IsolatedSMC_U.Vx) *
    30.0;
  if (rtb_errore < 0.0) {
    rtb_errore = -1.0;
  } else if (rtb_errore > 0.0) {
    rtb_errore = 1.0;
  } else if (rtb_errore == 0.0) {
    rtb_errore = 0.0;
  } else {
    rtb_errore = (rtNaN);
  }

  // Outport: '<Root>/Vx_cmd' incorporates:
  //   MATLAB Function: '<Root>/MATLAB Function1'

  IsolatedSMC_Y.Vx_cmd = 0.1 * rtb_errore;

  // Stop: '<Root>/Stop Simulation' incorporates:
  //   Constant: '<S5>/Constant'
  //   Inport: '<Root>/eos'
  //   RelationalOperator: '<S5>/Compare'

  if (!(IsolatedSMC_U.eos == 0.0)) {
    rtmSetStopRequested((&IsolatedSMC_M), 1);
  }

  // End of Stop: '<Root>/Stop Simulation'

  // SampleTimeMath: '<S3>/TSamp' incorporates:
  //   Inport: '<Root>/psi_ref'
  //
  //  About '<S3>/TSamp':
  //   y = u * K where K = 1 / ( w * Ts )

  rtb_TSamp_p0 = IsolatedSMC_U.psi_ref * 100.0;

  // SampleTimeMath: '<S4>/TSamp' incorporates:
  //   Inport: '<Root>/psi'
  //
  //  About '<S4>/TSamp':
  //   y = u * K where K = 1 / ( w * Ts )

  rtb_TSamp_h = IsolatedSMC_U.psi * 100.0;

  // MATLAB Function: '<Root>/MATLAB Function' incorporates:
  //   Gain: '<Root>/Gain'
  //   Gain: '<Root>/Gain2'
  //   Inport: '<Root>/psi'
  //   Inport: '<Root>/psi_ref'

  rtb_errore = 57.295779513082323 * IsolatedSMC_U.psi_ref - 57.295779513082323 *
    IsolatedSMC_U.psi;
  if (std::abs(rtb_errore) > 180.0) {
    if (rtb_errore < 0.0) {
      rtb_errore_0 = -1.0;
    } else if (rtb_errore > 0.0) {
      rtb_errore_0 = 1.0;
    } else if (rtb_errore == 0.0) {
      rtb_errore_0 = 0.0;
    } else {
      rtb_errore_0 = (rtNaN);
    }

    rtb_errore -= 360.0 * rtb_errore_0;
  }

  // End of MATLAB Function: '<Root>/MATLAB Function'

  // MATLAB Function: '<Root>/MATLAB Function2' incorporates:
  //   Sum: '<Root>/Sum2'
  //   Sum: '<S3>/Diff'
  //   Sum: '<S4>/Diff'
  //   UnitDelay: '<S3>/UD'
  //   UnitDelay: '<S4>/UD'
  //
  //  Block description for '<S3>/Diff':
  //
  //   Add in CPU
  //
  //  Block description for '<S4>/Diff':
  //
  //   Add in CPU
  //
  //  Block description for '<S3>/UD':
  //
  //   Store in Global RAM
  //
  //  Block description for '<S4>/UD':
  //
  //   Store in Global RAM

  rtb_errore = ((rtb_TSamp_p0 - IsolatedSMC_DW.UD_DSTATE_g) - (rtb_TSamp_h -
    IsolatedSMC_DW.UD_DSTATE_b)) + 100.0 * rtb_errore;
  if (rtb_errore < 0.0) {
    rtb_errore = -1.0;
  } else if (rtb_errore > 0.0) {
    rtb_errore = 1.0;
  } else if (rtb_errore == 0.0) {
    rtb_errore = 0.0;
  } else {
    rtb_errore = (rtNaN);
  }

  // Outport: '<Root>/psi_cmd' incorporates:
  //   MATLAB Function: '<Root>/MATLAB Function2'

  IsolatedSMC_Y.psi_cmd = 0.2 * rtb_errore;

  // Update for UnitDelay: '<S1>/UD'
  //
  //  Block description for '<S1>/UD':
  //
  //   Store in Global RAM

  IsolatedSMC_DW.UD_DSTATE = rtb_TSamp;

  // Update for UnitDelay: '<S2>/UD'
  //
  //  Block description for '<S2>/UD':
  //
  //   Store in Global RAM

  IsolatedSMC_DW.UD_DSTATE_o = rtb_TSamp_o;

  // Update for UnitDelay: '<S3>/UD'
  //
  //  Block description for '<S3>/UD':
  //
  //   Store in Global RAM

  IsolatedSMC_DW.UD_DSTATE_g = rtb_TSamp_p0;

  // Update for UnitDelay: '<S4>/UD'
  //
  //  Block description for '<S4>/UD':
  //
  //   Store in Global RAM

  IsolatedSMC_DW.UD_DSTATE_b = rtb_TSamp_h;
}

// Model initialize function
void IsolatedSMCModelClass::initialize()
{
  // Registration code

  // initialize non-finites
  rt_InitInfAndNaN(sizeof(real_T));
}

// Model terminate function
void IsolatedSMCModelClass::terminate()
{
  // (no terminate code required)
}

// Constructor
IsolatedSMCModelClass::IsolatedSMCModelClass() :
  IsolatedSMC_DW(),
  IsolatedSMC_U(),
  IsolatedSMC_Y(),
  IsolatedSMC_M()
{
  // Currently there is no constructor body generated.
}

// Destructor
IsolatedSMCModelClass::~IsolatedSMCModelClass()
{
  // Currently there is no destructor body generated.
}

// Real-Time Model get method
IsolatedSMCModelClass::RT_MODEL_IsolatedSMC_T * IsolatedSMCModelClass::getRTM()
{
  return (&IsolatedSMC_M);
}

//
// File trailer for generated code.
//
// [EOF]
//
