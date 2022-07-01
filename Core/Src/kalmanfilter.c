#include <stdio.h>
#include <string.h>
#include <math.h>
#include "kalmanfilter.h"
#include "mathlib.h"

void kf(float dt, float F[NUM_S][NUM_S], float B[NUM_S][NUM_I], float H[NUM_S][NUM_S], \
        float x_est[NUM_S][1], float P[NUM_S][NUM_S], float Q[NUM_S][NUM_S], float R[NUM_S][NUM_S],\
        float u_p[NUM_I][1], float z_p_kf[NUM_S][1], float x_est_update[NUM_S][1], float P_update[NUM_S][NUM_S])
{

    // temp variables
    float F_x_x_est[NUM_S][1];
    float B_x_u[NUM_S][1];
    float x_est_p[NUM_S][1];
    float F_t[NUM_S][NUM_S];
    float F_x_P[NUM_S][NUM_S];
    float FP_x_Ft[NUM_S][NUM_S];
    float P_p[NUM_S][NUM_S];
    float H_x_x_est[NUM_S][NUM_S];
    float y[NUM_S][1];
    float H_t[NUM_S][NUM_S];
    float H_x_P[NUM_S][NUM_S];
    float HP_x_Ht[NUM_S][NUM_S];
    float R_s[NUM_S][NUM_S];
    float Rs_inv[NUM_S][NUM_S];
    float P_x_H[NUM_S][NUM_S];
    float k_gain[NUM_S][NUM_S];
    float k_x_y[NUM_S][1];
    float iden[NUM_S][NUM_S];
    float k_x_H[NUM_S][NUM_S];
    float i_kH[NUM_S][NUM_S];

    // predicted state estimate
    mulMat(NUM_S, NUM_S, F, NUM_S, 1, x_est, F_x_x_est);
    mulMat(NUM_S, NUM_I, B, NUM_I, 1, u_p, B_x_u);
    addMat(NUM_S, 1, F_x_x_est, B_x_u, x_est_p);
    // predicted error covariance
    transposeMat(NUM_S, NUM_S, F, F_t);
    mulMat(NUM_S, NUM_S, F, NUM_S, NUM_S, P, F_x_P);
    mulMat(NUM_S, NUM_S, F_x_P, NUM_S, NUM_S, F_t, FP_x_Ft);
    addMat(NUM_S, NUM_S, FP_x_Ft, Q, P_p);
    // Update

    // measurement residual
    mulMat(NUM_S, NUM_S, H, NUM_S, 1, x_est_p, H_x_x_est);
    subMat(NUM_S, 1, z_p_kf, H_x_x_est, y);

    // Kalman gain
    transposeMat(NUM_S, NUM_S, H, H_t);
    mulMat(NUM_S, NUM_S, H, NUM_S, NUM_S, P_p, H_x_P);
    mulMat(NUM_S, NUM_S, H_x_P, NUM_S, NUM_S, H_t, HP_x_Ht);
    addMat(NUM_S, NUM_S, HP_x_Ht, R, R_s);
    inverseMat(NUM_S, R_s, Rs_inv);
    mulMat(NUM_S, NUM_S, P_p, NUM_S, NUM_S, H_t, P_x_H);
    mulMat(NUM_S, NUM_S, P_x_H, NUM_S, NUM_S, Rs_inv, k_gain);

    // updated state estimate
    mulMat(NUM_S, NUM_S, k_gain, NUM_S, 1, y, k_x_y);
    addMat(NUM_S, NUM_S, x_est_p, k_x_y, x_est_update);

    // updated error covariance
    mulMat(NUM_S, NUM_S, k_gain, NUM_S, NUM_S, H, k_x_H);
    identityMat(NUM_S, iden);
    subMat(NUM_S, NUM_S, iden, k_x_H, i_kH);
    mulMat(NUM_S, NUM_S, i_kH, NUM_S, NUM_S, P_p, P_update);
}
