#ifndef LQR_UTILS_H_
#define LQR_UTILS_H_
#include "stabilizer_types.h"

#define RAD2DEG 57.29577951308232087679f
#define DEG2RAD 0.01745329251994329577f
#define F_HOVER 0.0907425f
#define F_MAX 0.15

void createRotationMatrix(double roll, double pitch, double yaw, double R[3][3]);
void matrixMultiply(int rowsA, int colsA, int colsB,
                    const float (*A)[rowsA][colsA], const float (*B)[colsA][colsB], float (*C)[rowsA][colsB]);
void subtractMatrices(int rows, int cols, const float (*A)[rows][cols], const float (*B)[rows][cols], float (*Result)[rows][cols]);


void minusMatrix(int rows, int cols, float (*A)[rows][cols]);
void createStateVector(float (*stateVect)[12][1], const state_t *state, const sensorData_t *sensors);
void createRefVector(float (*refVect)[12][1], const setpoint_t *setpoint);

void clipMatrix(int rows, int cols, float minVal, float maxVal, float (*matrix)[rows][cols]);
void normalize(int rows, int cols, float maxVal, float (*matrix)[rows][cols]);
#endif /* LQR_UTILS_H_ */