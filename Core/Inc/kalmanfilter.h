/* kalmanfilter.h */

#ifndef __KALMANFILTER_H__
#define __KALMANFILTER_H__

#define NUM_S 6
#define NUM_I 3

void kf(float dt, float F[NUM_S][NUM_S], float B[NUM_S][NUM_I], float H[NUM_S][NUM_S], \
        float x_est[NUM_S][1], float P[NUM_S][NUM_S], float Q[NUM_S][NUM_S], float R[NUM_S][NUM_S],\
        float u_p[NUM_I][1], float z_p_kf[NUM_S][1], float x_est_update[NUM_S][1], float P_update[NUM_S][NUM_S]);

#endif /* __KALMANFILTER_H__ */
