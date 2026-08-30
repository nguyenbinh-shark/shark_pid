/**
 * @file  shark_pid_sfun_common.h
 * @brief Phan dung chung cua hai S-function boc shark_pid: kiem tra tham so
 *        dialog, doi ma so cua hai o combo box sang co, va cap phat trang thai.
 *
 * Chi include SAU khi da dinh nghia S_FUNCTION_NAME / S_FUNCTION_LEVEL, vi
 * file nay keo theo "simstruc.h".
 *
 * Hai o combo box cua khoi PID (2DOF) khong the truyen bang so thuc "tu do"
 * duoc, nen chung duoc ma hoa thanh so nguyen — xem hai enum ben duoi. Truoc
 * day hai gia tri nay bi VIET CHET trong mdlStart (luon Trapezoidal + clamping)
 * nen khong the doi chieu cau hinh Backward Euler hay back-calculation qua
 * S-function. Bay gio chung la tham so dialog.
 */
#ifndef SHARK_PID_SFUN_COMMON_H
#define SHARK_PID_SFUN_COMMON_H

#include "simstruc.h"
#include "shark_pid.h"
#include <stdlib.h>

/** Gia tri o tham so thu i, kieu real_T. */
#define SP_PVAL(S, i) ((real_T)mxGetScalar(ssGetSFcnParam((S), (i))))

/** Ma hoa o `Integrator method`. */
enum {
    SP_IM_BACKWARD_EULER = 0,
    SP_IM_TRAPEZOIDAL    = 1
};

/** Ma hoa o `Anti-windup method`. */
enum {
    SP_AW_NONE     = 0,
    SP_AW_CLAMPING = 1,
    SP_AW_BACKCALC = 2
};

/**
 * Doi hai ma so tren sang cfg.flags.
 * Khoi PID chi chon duoc MOT phuong phap chong windup, nen o day cung vay.
 */
static uint32_t sp_sfun_flags(real_T imethod, real_T awmethod)
{
    uint32_t f = 0u;

    if ((int)imethod == SP_IM_TRAPEZOIDAL) {
        f |= (uint32_t)SHARK_PID_F_TRAPEZOID_I;
    }
    if ((int)awmethod == SP_AW_CLAMPING) {
        f |= (uint32_t)SHARK_PID_F_CLAMP_I;
    } else if ((int)awmethod == SP_AW_BACKCALC) {
        f |= (uint32_t)SHARK_PID_F_BACKCALC_I;
    }
    return f;
}

/**
 * Do phan cau hinh giong nhau o ca hai S-function vao cfg.
 * @param ts Sample time cua khoi — cung la dt truyen cho shark_pid_update().
 */
static void sp_sfun_fill_cfg(shark_pid_cfg_t *cfg,
                             real_T ts,
                             real_T out_min, real_T out_max,
                             real_T i_min,   real_T i_max,
                             real_T kb,
                             real_T imethod, real_T awmethod)
{
    cfg->out_min = (float)out_min;
    cfg->out_max = (float)out_max;
    cfg->i_min   = (float)i_min;
    cfg->i_max   = (float)i_max;
    cfg->kb      = (float)kb;

    /* Simulink cap dt deu tuyet doi -> khong de bo bao ve dt xen vao. */
    cfg->dt_max     = 1.0e3f;
    cfg->dt_nominal = (float)ts;

    cfg->flags = sp_sfun_flags(imethod, awmethod);
}

/** Cap phat trang thai cho mot instance. NULL kem ErrorStatus neu that bai. */
static shark_pid_t *sp_sfun_alloc(SimStruct *S)
{
    shark_pid_t *pid = (shark_pid_t *)calloc(1, sizeof(shark_pid_t));

    if (pid == NULL) {
        ssSetErrorStatus(S, "shark_pid: khong cap phat duoc bo nho");
    }
    return pid;
}

/** Giai phong trang thai trong mdlTerminate. An toan khi goi nhieu lan. */
static void sp_sfun_free(SimStruct *S)
{
    void **pw = ssGetPWork(S);

    if (pw != NULL && pw[0] != NULL) {
        free(pw[0]);
        pw[0] = NULL;
    }
}

#ifdef MATLAB_MEX_FILE
/**
 * Moi o tham so phai la mot so thuc huu han; rieng Ts phai duong.
 * Tra ve NULL neu hop le, nguoc lai tra ve thong bao (chuoi hang -> ton tai
 * du lau cho ssSetErrorStatus).
 * @param i_ts Chi so cua o Sample time, hoac -1 neu khong co o nao la Ts.
 */
static const char *sp_sfun_check_params(SimStruct *S, int_T nparams, int_T i_ts)
{
    int_T i;

    for (i = 0; i < nparams; i++) {
        const mxArray *p = ssGetSFcnParam(S, i);

        if (p == NULL || !mxIsNumeric(p) || mxIsComplex(p) ||
            mxGetNumberOfElements(p) != 1) {
            return "shark_pid: moi tham so phai la mot so thuc vo huong";
        }
        if (!mxIsFinite(mxGetScalar(p))) {
            return "shark_pid: tham so khong duoc la NaN/Inf "
                   "(muon 'khong gioi han' thi dung mot so lon, vi du 1e6)";
        }
    }
    if (i_ts >= 0 && i_ts < nparams && SP_PVAL(S, i_ts) <= 0.0) {
        return "shark_pid: Sample time phai duong (khoi khong ke thua duoc)";
    }
    return NULL;
}
#endif /* MATLAB_MEX_FILE */

#endif /* SHARK_PID_SFUN_COMMON_H */
