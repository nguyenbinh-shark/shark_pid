/**
 * @file  shark_pid_sfun.c
 * @brief Level-2 C MEX S-function boc loi C99 shark_pid, de so sanh truc tiep
 *        voi khoi Simulink "PID Controller (2DOF)".
 *
 * Diem cot loi: Simulink goi THANG ma C se nap len vi dieu khien, khong phai
 * mot ban mo phong viet lai bang MATLAB.
 *
 * Build (chay tu verify_shark_vs_pid2.m, hoac bang tay):
 *   SRC = fullfile(pwd,'..','..','src');
 *   mex(['-I' SRC], '-I.', 'shark_pid_sfun.c', fullfile(SRC,'shark_pid.c'));
 *   % Linux/macOS: them 'CFLAGS=$CFLAGS -std=c99' vao dau danh sach.
 *
 * Cong / Ports:  In1 = r (setpoint), In2 = y (measurement), Out1 = u
 *
 * Tham so dialog — dung thu tu enum ben duoi:
 *   Ts, kp, ki, kd, b, c, n, out_min, out_max, i_min, i_max,
 *   imethod (0 = Backward Euler, 1 = Trapezoidal),
 *   awmethod (0 = none, 1 = clamping, 2 = back-calculation), kb
 *
 * Ba o cuoi la phan them cua ban nay: truoc day Integrator method va
 * Anti-windup method bi viet chet trong mdlStart nen khong doi chieu duoc hai
 * cau hinh dang gia nhat cua 2.0 — Backward Euler va back-calculation. Da co
 * .slx cu thi phai sua lai chuoi Parameters cho du 14 o (hoac chay lai
 * make_shark_mask).
 */

#define S_FUNCTION_NAME  shark_pid_sfun
#define S_FUNCTION_LEVEL 2

#include "shark_pid_sfun_common.h"

/* Thu tu tham so trong dialog S-Function */
enum {
    P_TS = 0,   /* sample time [s]                              */
    P_KP,       /* kp                                            */
    P_KI,       /* ki  [1/s]  (= I cua Simulink)                 */
    P_KD,       /* kd  [s]    (= D cua Simulink)                 */
    P_B,        /* setpoint weight b                             */
    P_C,        /* setpoint weight c                             */
    P_N,        /* n  [rad/s] (= Filter coefficient N cua khoi)  */
    P_OUTMIN,
    P_OUTMAX,
    P_IMIN,
    P_IMAX,
    P_IMETHOD,  /* 0 = Backward Euler, 1 = Trapezoidal           */
    P_AWMETHOD, /* 0 = none, 1 = clamping, 2 = back-calculation  */
    P_KB,       /* Back-calculation coefficient (Kb) [1/s]       */
    NPARAMS
};

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
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
        return;  /* Simulink bao loi so tham so */
    }
#ifdef MATLAB_MEX_FILE
    mdlCheckParameters(S);
    if (ssGetErrorStatus(S) != NULL) return;
#endif
    for (i = 0; i < NPARAMS; i++) {
        ssSetSFcnParamTunable(S, i, SS_PRM_NOT_TUNABLE);
    }

    ssSetNumContStates(S, 0);
    ssSetNumDiscStates(S, 0);   /* trang thai nam trong shark_pid_t (PWork) */

    if (!ssSetNumInputPorts(S, 2)) return;
    for (i = 0; i < 2; i++) {
        ssSetInputPortWidth(S, i, 1);
        ssSetInputPortRequiredContiguous(S, i, 1);
        ssSetInputPortDirectFeedThrough(S, i, 1);  /* giong khoi PID that */
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

    /* Moi dong duoi day la dung mot o trong Block Parameters cua khoi 2DOF. */
    cfg.kp = (float)SP_PVAL(S, P_KP);
    cfg.ki = (float)SP_PVAL(S, P_KI);
    cfg.kd = (float)SP_PVAL(S, P_KD);
    cfg.b  = (float)SP_PVAL(S, P_B);
    cfg.c  = (float)SP_PVAL(S, P_C);
    cfg.n  = (float)SP_PVAL(S, P_N);        /* o Filter coefficient (N) */

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
    shark_pid_t   *pid = (shark_pid_t *)ssGetPWork(S)[0];
    const real_T  *r   = (const real_T *)ssGetInputPortSignal(S, 0);
    const real_T  *y   = (const real_T *)ssGetInputPortSignal(S, 1);
    real_T        *u   = ssGetOutputPortRealSignal(S, 0);

    (void)tid;

    /* shark_pid_update() vua tinh ngo ra vua cap nhat trang thai -> chi duoc
       goi MOT lan moi buoc lon. Dung solver Fixed-step Discrete.
       dt lay tu cfg.dt_nominal (= o Ts) nen khong phai tra ve dialog moi buoc,
       va chac chan trung dung gia tri ma mdlStart da cau hinh. */
    if (ssIsMajorTimeStep(S)) {
        u[0] = (real_T)shark_pid_update(pid, (float)r[0], (float)y[0],
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
