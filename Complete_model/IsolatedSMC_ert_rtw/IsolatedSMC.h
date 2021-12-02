//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: IsolatedSMC.h
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
#ifndef RTW_HEADER_IsolatedSMC_h_
#define RTW_HEADER_IsolatedSMC_h_
#include <cmath>
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#include "IsolatedSMC_types.h"
#include "rtGetNaN.h"
#include "rt_nonfinite.h"
#include "rtGetInf.h"

// Macros for accessing real-time model data structure
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

#ifndef rtmGetStopRequested
#define rtmGetStopRequested(rtm)       ((rtm)->Timing.stopRequestedFlag)
#endif

#ifndef rtmSetStopRequested
#define rtmSetStopRequested(rtm, val)  ((rtm)->Timing.stopRequestedFlag = (val))
#endif

#ifndef rtmGetStopRequestedPtr
#define rtmGetStopRequestedPtr(rtm)    (&((rtm)->Timing.stopRequestedFlag))
#endif

// Class declaration for model IsolatedSMC
class IsolatedSMCModelClass {
  // public data and function members
 public:
  // Block states (default storage) for system '<Root>'
  typedef struct {
    real_T UD_DSTATE;                  // '<S1>/UD'
    real_T UD_DSTATE_o;                // '<S2>/UD'
    real_T UD_DSTATE_g;                // '<S3>/UD'
    real_T UD_DSTATE_b;                // '<S4>/UD'
  } DW_IsolatedSMC_T;

  // External inputs (root inport signals with default storage)
  typedef struct {
    real_T Vx_ref;                     // '<Root>/Vx_ref'
    real_T psi_ref;                    // '<Root>/psi_ref'
    real_T psi;                        // '<Root>/psi'
    real_T Vx;                         // '<Root>/Vx'
    real_T eos;                        // '<Root>/eos'
  } ExtU_IsolatedSMC_T;

  // External outputs (root outports fed by signals with default storage)
  typedef struct {
    real_T Vx_cmd;                     // '<Root>/Vx_cmd'
    real_T psi_cmd;                    // '<Root>/psi_cmd'
  } ExtY_IsolatedSMC_T;

  // Real-time Model Data Structure
  struct RT_MODEL_IsolatedSMC_T {
    const char_T * volatile errorStatus;

    //
    //  Timing:
    //  The following substructure contains information regarding
    //  the timing information for the model.

    struct {
      boolean_T stopRequestedFlag;
    } Timing;
  };

  // model initialize function
  void initialize();

  // model step function
  void step();

  // model terminate function
  void terminate();

  // Constructor
  IsolatedSMCModelClass();

  // Destructor
  ~IsolatedSMCModelClass();

  // Root-level structure-based inputs set method

  // Root inports set method
  void setExternalInputs(const ExtU_IsolatedSMC_T* pExtU_IsolatedSMC_T)
  {
    IsolatedSMC_U = *pExtU_IsolatedSMC_T;
  }

  // Root-level structure-based outputs get method

  // Root outports get method
  const IsolatedSMCModelClass::ExtY_IsolatedSMC_T & getExternalOutputs() const
  {
    return IsolatedSMC_Y;
  }

  // Real-Time Model get method
  IsolatedSMCModelClass::RT_MODEL_IsolatedSMC_T * getRTM();

  // private data and function members
 private:
  // Block states
  DW_IsolatedSMC_T IsolatedSMC_DW;

  // External inputs
  ExtU_IsolatedSMC_T IsolatedSMC_U;

  // External outputs
  ExtY_IsolatedSMC_T IsolatedSMC_Y;

  // Real-Time Model
  RT_MODEL_IsolatedSMC_T IsolatedSMC_M;
};

//-
//  These blocks were eliminated from the model due to optimizations:
//
//  Block '<S1>/Data Type Duplicate' : Unused code path elimination
//  Block '<S2>/Data Type Duplicate' : Unused code path elimination
//  Block '<S3>/Data Type Duplicate' : Unused code path elimination
//  Block '<S4>/Data Type Duplicate' : Unused code path elimination
//  Block '<Root>/Rate Transition' : Unused code path elimination
//  Block '<Root>/Rate Transition1' : Unused code path elimination
//  Block '<Root>/Scope' : Unused code path elimination
//  Block '<Root>/Scope1' : Unused code path elimination
//  Block '<Root>/Scope2' : Unused code path elimination
//  Block '<Root>/To Workspace1' : Unused code path elimination
//  Block '<Root>/To Workspace5' : Unused code path elimination


//-
//  The generated code includes comments that allow you to trace directly
//  back to the appropriate location in the model.  The basic format
//  is <system>/block_name, where system is the system number (uniquely
//  assigned by Simulink) and block_name is the name of the block.
//
//  Use the MATLAB hilite_system command to trace the generated code back
//  to the model.  For example,
//
//  hilite_system('<S3>')    - opens system 3
//  hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
//
//  Here is the system hierarchy for this model
//
//  '<Root>' : 'IsolatedSMC'
//  '<S1>'   : 'IsolatedSMC/Discrete Derivative'
//  '<S2>'   : 'IsolatedSMC/Discrete Derivative1'
//  '<S3>'   : 'IsolatedSMC/Discrete Derivative2'
//  '<S4>'   : 'IsolatedSMC/Discrete Derivative3'
//  '<S5>'   : 'IsolatedSMC/IsNonZero'
//  '<S6>'   : 'IsolatedSMC/IsZero'
//  '<S7>'   : 'IsolatedSMC/MATLAB Function'
//  '<S8>'   : 'IsolatedSMC/MATLAB Function1'
//  '<S9>'   : 'IsolatedSMC/MATLAB Function2'

#endif                                 // RTW_HEADER_IsolatedSMC_h_

//
// File trailer for generated code.
//
// [EOF]
//
