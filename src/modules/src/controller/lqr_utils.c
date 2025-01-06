#include <math.h>
#include "lqr_utils.h"
#include "math3d.h"




void createRotationMatrix(double roll, double pitch, double yaw, double R[3][3]) {
    // Compute trigonometric values
    double cr = cos(roll);  // Cosine of roll
    double sr = sin(roll);  // Sine of roll
    double cp = cos(pitch); // Cosine of pitch
    double sp = sin(pitch); // Sine of pitch
    double cy = cos(yaw);   // Cosine of yaw
    double sy = sin(yaw);   // Sine of yaw

    // Compute the rotation matrix
    R[0][0] = cy * cp;
    R[0][1] = cy * sp * sr - sy * cr;
    R[0][2] = cy * sp * cr + sy * sr;

    R[1][0] = sy * cp;
    R[1][1] = sy * sp * sr + cy * cr;
    R[1][2] = sy * sp * cr - cy * sr;

    R[2][0] = -sp;
    R[2][1] = cp * sr;
    R[2][2] = cp * cr;
}


void matrixMultiply(int rowsA, int colsA, int colsB,
                    const float (*A)[rowsA][colsA], const float (*B)[colsA][colsB], float (*C)[rowsA][colsB]) {
    // Initialize the result matrix C to zero
    for (int i = 0; i < rowsA; i++) {
        for (int j = 0; j < colsB; j++) {
            (*C)[i][j] = 0.0;
        }
    }

    // Perform multiplication
    for (int i = 0; i < rowsA; i++) {
        for (int j = 0; j < colsB; j++) {
            for (int k = 0; k < colsA; k++) {
                (*C)[i][j] += (*A)[i][k] * (*B)[k][j];
            }
        }
    }
}


void subtractMatrices(int rows, int cols, const float (*A)[rows][cols], const float (*B)[rows][cols], float (*Result)[rows][cols]) {
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            (*Result)[i][j] = (*A)[i][j] - (*B)[i][j];
        }
    }
}


void minusMatrix(int rows, int cols, float (*A)[rows][cols])
{

    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            (*A)[i][j] = -(*A)[i][j];
        }
    }

}


void createStateVector(float (*stateVect)[12][1], const state_t *state, const sensorData_t *sensors)
{
    (*stateVect)[0][0] = state->position.x;
    (*stateVect)[1][0] = state->position.y;
    (*stateVect)[2][0] = state->position.z;
    (*stateVect)[3][0] = state->velocity.x;
    (*stateVect)[4][0] = state->velocity.y;
    (*stateVect)[5][0] = state->velocity.z;
    struct quat setpoint_quat = mkquat(state->attitudeQuaternion.x, state->attitudeQuaternion.y, state->attitudeQuaternion.z, state->attitudeQuaternion.w);
    struct vec rpy = quat2rpy(setpoint_quat);
    (*stateVect)[6][0] = rpy.x;
    (*stateVect)[7][0] = rpy.y;
    (*stateVect)[8][0] = rpy.z;
    (*stateVect)[9][0] = DEG2RAD * sensors->gyro.x;
    (*stateVect)[10][0] = DEG2RAD * sensors->gyro.y;
    (*stateVect)[11][0] = DEG2RAD * sensors->gyro.z;
}

void createRefVector(float (*refVect)[12][1], const setpoint_t *setpoint)
{
    for(int i = 0; i < 12; i++)
    {
        (*refVect)[i][0] = 0.0f;
    }
    (*refVect)[0][0] = setpoint->position.x;
    (*refVect)[1][0] = setpoint->position.y;
    (*refVect)[2][0] = setpoint->position.z;

}



void clipMatrix(int rows, int cols, float minVal, float maxVal, float (*matrix)[rows][cols]) {
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            // Calculate the index in the 1D array
            
            // Clip the matrix element at matrix[i][j] to be between minVal and maxVal
            if ((*matrix)[i][j] < minVal) {
                (*matrix)[i][j] = minVal;
            } else if ((*matrix)[i][j] > maxVal) {
                (*matrix)[i][j] = maxVal;
            }
        }
    }
}


void normalize(int rows, int cols, float maxVal, float (*matrix)[rows][cols])
{
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
           (*matrix)[i][j] /= maxVal; 
        }
    }
}
