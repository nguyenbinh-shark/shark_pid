/**
 * @file  shark_pid_sfun_tunable.c
 * @brief Ban S-function co HE SO LAM CONG VAO - chinh duoc trong luc chay.
 *        Dung cho gain scheduling / auto-tuning / thu bumpless gain change.
 *
 * Build:
 *   SRC = fullfile(pwd,'..','..','src');
 *   mex(['-I' SRC], '-I.', 'shark_pid_sfun_tunable.c', fullfile(SRC,'shark_pid.c'));
 *
 * Cong vao / Input ports:
 *   1: r      (setpoint)        5: kd     [s]
 *   2: y      (measurement)     6: b      (setpoint weight, khau P)
 *   3: kp                       7: c      (setpoint weight, khau D)
 *   4: ki     [1/s]             8: n      [rad/s]  (Filter coefficient N)
 * Cong ra: u
 *
 * Tham so dialog: Ts, out_min, out_max, i_min, i_max, imethod, awmethod, kb
 *   imethod  0 = Backward Euler, 1 = Trapezoidal
 *   awmethod 0 = none, 1 = clamping, 2 = back-calculation
 *
 * LUU Y ve doi he so luc dang chay:
 *   shark_pid luu i_state = ki*Integral(e) (DA nhan ki), nen doi ki khong hoi to
 *   len lich su -> ngo ra khong nhay bac. Day chinh la tinh chat can quan sat.
 *   NHUNG shark_pid_set_gains() xoa bo tich phan khi ki == 0. Neu quet ki qua 0
 *   thi bo tich phan bi reset - do la hanh vi co chu y cua thu vien, khong
 *   phai loi cua S-function.
 */

#define S_FUNCTION_NAME  shark_pid_sfun_tunable
#define S_FUNCTION_LEVEL 2

#include "shark_pid_sfun_common.h"

/* thu tu tham so dialog */
enum {
    P_TS = 0,
    P_OUTMIN,
    P_OUTMAX,
    P_IMIN,
    P_IMAX,
    P_IMETHOD,
    P_AWMETHOD,
    P_KB,
    NPARAMS
};

/* thu tu cong vao */
enum { IN_R = 0, IN_Y, IN_KP, IN_KI, IN_KD, IN_B, IN_C, IN_N, NINPUTS };

#define UVAL(S, i) (*(const real_T *)ssGetInputPortSignal((S), (i)))

/* ------------------------------------------------------------------ */
#ifdef MATLAB_MEX_FILE
#define MDL_CHECK_PARAMETERS
static void mdlCheckParameters(SimStruct *S)
{
    const char *msg = sp_sfun_check_params(S, NPARAMS, P_TS);
    if (msg != NULL) {
        ssSetErrorStatus(S, msg);
    }
}
#endif

/* ------------------------------------------------------------------ */
static void mdlInitializeSizes(SimStruct *S)
{
    int_T i;

    ssSetNumSFcnParams(S, NPARAMS);
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) return;
#ifdef MATLAB_MEX_FILE
    mdlCheckParameters(S);
    if (ssGetErrorStatus(S) != NULL) return;
#endif
    for (i = 0; i < NPARAMS; i++) {
        ssSetSFcnParamTunable(S, i, SS_PRM_NOT_TUNABLE);
    }

    ssSetNumContStates(S, 0);
    ssSetNumDiscStates(S, 0);

    if (!ssSetNumInputPorts(S, NINPUTS)) return;
    for (i = 0; i < NINPUTS; i++) {
        ssSetInputPortWidth(S, i, 1);
        ssSetInputPortRequiredContiguous(S, i, 1);
        ssSetInputPortDirectFeedThrough(S, i, 1);
    }

    if (!ssSetNumOutputPorts(S, 1)) return;
    ssSetOutputPortWidth(S, 0, 1);

    ssSetNumSampleTimes(S, 1);
    ssSetNumPWork(S, 1);

    ssSetOptions(S, SS_OPTION_EXCEPTION_FREE_CODE |
                    SS_OPTION_CALL_TERMINATE_ON_EXIT);
}

/* ------------------------------------------------------------------ */
static void mdlInitializeSampleTimes(SimStruct *S)
{
    ssSetSampleTime(S, 0, SP_PVAL(S, P_TS));
    ssSetOffsetTime(S, 0, 0.0);
    ssSetModelReferenceSampleTimeDefaultInheritance(S);
}

/* ------------------------------------------------------------------ */
#define MDL_START
static void mdlStart(SimStruct *S)
{
    shark_pid_cfg_t cfg;
    shark_pid_t *pid = sp_sfun_alloc(S);

    if (pid == NULL) return;

    shark_pid_cfg_default(&cfg);

    /* he so se duoc ghi de moi buoc tu cong vao; dat tam gia tri an toan */
    cfg.kp = 0.0f;  cfg.ki = 0.0f;  cfg.kd = 0.0f;
    cfg.b  = 1.0f;  cfg.c  = 0.0f;  cfg.n  = 100.0f;

    sp_sfun_fill_cfg(&cfg, SP_PVAL(S, P_TS),
                     SP_PVAL(S, P_OUTMIN), SP_PVAL(S, P_OUTMAX),
                     SP_PVAL(S, P_IMIN),   SP_PVAL(S, P_IMAX),
                     SP_PVAL(S, P_KB),
                     SP_PVAL(S, P_IMETHOD), SP_PVAL(S, P_AWMETHOD));

    shark_pid_init(pid, &cfg);
    ssGetPWork(S)[0] = (void *)pid;
}

/* ------------------------------------------------------------------ */
static void mdlOutputs(SimStruct *S, int_T tid)
{
    shark_pid_t *pid = (shark_pid_t *)ssGetPWork(S)[0];
    real_T      *u   = ssGetOutputPortRealSignal(S, 0);

    (void)tid;

    if (ssIsMajorTimeStep(S)) {
        /* --- nap he so tu cong vao truoc moi buoc tinh --- */
        /* set_gains(): i_state giu nguyen -> doi he so KHONG lam nhay ngo ra */
        shark_pid_set_gains(pid, (float)UVAL(S, IN_KP),
                                 (float)UVAL(S, IN_KI),
                                 (float)UVAL(S, IN_KD));
        /* b, c, n khong co setter rieng -> ghi thang vao cfg */
        pid->cfg.b = (float)UVAL(S, IN_B);
        pid->cfg.c = (float)UVAL(S, IN_C);
        pid->cfg.n = (float)UVAL(S, IN_N);

        u[0] = (real_T)shark_pid_update(pid, (float)UVAL(S, IN_R),
                                             (float)UVAL(S, IN_Y),
                                             pid->cfg.dt_nominal);
    } else {
        u[0] = (real_T)pid->output;
    }
}

/* ------------------------------------------------------------------ */
static void mdlTerminate(SimStruct *S)
{
    sp_sfun_free(S);
}

#ifdef MATLAB_MEX_FILE
#include "simulink.c"
#else
#include "cg_sfun.h"
#endif
