#include <stdlib.h>

#include "stabilizer_types.h"

#include "lqr_utils.h"
#include "controller_lqr.h"


#include "log.h"
#include "param.h"
#include "debug.h"

#define ATTITUDE_UPDATE_DT    (float)(1.0f/ATTITUDE_RATE)

float stateVect[12][1];
float refVect[12][1];
float controlState[12][1];
float controlVect[4][1];

float targetR[3][3];

float K[4][12] = {
    {-0.02236068, 0.02236068, 0.03535534, -0.01028867, 0.01028867, 0.03397166, -0.02322055, -0.02322055, -0.05000000, -0.00295620, -0.00295620, -0.01164389},
    {0.02236068, 0.02236068, 0.03535534, 0.00999996, 0.00999996, 0.03397166, -0.02193566, 0.02193566, 0.05000000, -0.00287325, 0.00287325, 0.01164389},
    {0.02236068, -0.02236068, 0.03535534, 0.01028867, -0.01028867, 0.03397166, 0.02322055, 0.02322055, -0.05000000, 0.00295620, 0.00295620, -0.01164389},
    {-0.02236068, -0.02236068, 0.03535534, -0.00999996, -0.00999996, 0.03397166, 0.02193566, -0.02193566, 0.05000000, 0.00287325, -0.00287325, 0.01164389}
};
// float K[4][12] = {
//     {-0.01118034, 0.01118034, 0.11180340, -0.00657166, 0.00657166, 0.04683335, -0.01894670, -0.01894670, -0.07905694, -0.00310333, -0.00310333, -0.01468404},
//     {0.01118034, 0.01118034, 0.11180340, 0.00846931, 0.00846931, 0.04683335, -0.02050084, 0.02050084, 0.07905694, -0.00277769, 0.00277769, 0.01468404},
//     {0.01118034, -0.01118034, 0.11180340, 0.00657166, -0.00657166, 0.04683335, 0.01894670, 0.01894670, -0.07905694, 0.00310333, 0.00310333, -0.01468404},
//     {-0.01118034, -0.01118034, 0.11180340, -0.00846931, -0.00846931, 0.04683335, 0.02050084, -0.02050084, 0.07905694, 0.00277769, -0.00277769, 0.01468404}
// };


// float K[4][12] = {
//     {-0.01118034, 0.01118034, 0.11180340, -0.00658162, 0.00658162, 0.04683335, -0.01900418, -0.01900418, -0.07905694, -0.00310681, -0.00310681, -0.01468404},
//     {0.01118034, 0.01118034, 0.11180340, 0.00846931, 0.00846931, 0.04683335, -0.02050084, 0.02050084, 0.07905694, -0.00277769, 0.00277769, 0.01468404},
//     {0.01118034, -0.01118034, 0.11180340, 0.00658162, -0.00658162, 0.04683335, 0.01900418, 0.01900418, -0.07905694, 0.00310681, 0.00310681, -0.01468404},
//     {-0.01118034, -0.01118034, 0.11180340, -0.00846931, -0.00846931, 0.04683335, 0.02050084, -0.02050084, 0.07905694, 0.00277769, -0.00277769, 0.01468404}
// };

bool isInit = false;


void controllerLQRInit(void)
{   
    isInit = true;
}

bool controllerLQRTest(void)
{
    return isInit;
}


int max_time = 1000;
int i = 0;
void controllerLQR(control_t *control, const setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const stabilizerStep_t stabilizerStep)
{

    control->controlMode = controlModeForce;

    if (RATE_DO_EXECUTE(RATE_MAIN_LOOP, stabilizerStep) && setpoint->mode.z != modeDisable)
    {
        createStateVector(&stateVect, state, sensors);
        createRefVector(&refVect, setpoint);

        
        if (stateVect[2][0] <= 0.25f && refVect[2][0] <= 0.25f)
        {
            control->normalizedForces[0] = 0;
            control->normalizedForces[1] = 0;
            control->normalizedForces[2] = 0;
            control->normalizedForces[3] = 0;
            return;
        }


        subtractMatrices(12, 1, &stateVect, &refVect, &controlState);
        createRotationMatrix(0, 0, refVect[8][0], &targetR);

        float correctedPose[3][1];
        float correctedVel[3][1];
        float stateVel[3][1];
        stateVel[0][0] = stateVect[3][0];
        stateVel[1][0] = stateVect[4][0];
        stateVel[2][0] = stateVect[5][0];


        matrixMultiply(3, 3, 1, &targetR, &controlState, &correctedPose);
        matrixMultiply(3, 3, 1, &targetR, &stateVel, &correctedVel);

        controlState[0][0] = correctedPose[0][0];
        controlState[1][0] = correctedPose[1][0];
        controlState[2][0] = correctedPose[2][0];

        controlState[3][0] = correctedVel[0][0];
        controlState[4][0] = correctedVel[1][0];
        controlState[5][0] = correctedVel[2][0];

        for(int i = 0; i != 12; i++)
        {
            controlState[i][0] = - controlState[i][0];
        }


        matrixMultiply(4, 12, 1, &K, &controlState, &controlVect);

        for(int i = 0; i < 4; i ++)
        {
            controlVect[i][0] += F_HOVER;
        }

        clipMatrix(4, 1, 0.0f, F_MAX, &controlVect);

        control->normalizedForces[0] = controlVect[0][0];
        control->normalizedForces[1] = controlVect[1][0];
        control->normalizedForces[2] = controlVect[2][0];
        control->normalizedForces[3] = controlVect[3][0];
        
    }

    // if(i == max_time)
    // {
    //     DEBUG_PRINT("pose: %f; %f; %f\n", (double)stateVect[0][0], (double)stateVect[1][0], (double)stateVect[2][0]);
    //     // DEBUG_PRINT("actual pose: %f; %f; %f\n", (double)state->position.x, (double)state->position.y, (double)state->position.z);

    //     // DEBUG_PRINT("vel: %f; %f; %f\n", (double)stateVect[3][0], (double)stateVect[4][0], (double)stateVect[5][0]);
    //     // DEBUG_PRINT("attitude: %f; %f; %f\n", (double)stateVect[6][0], (double)stateVect[7][0], (double)stateVect[8][0]);
    //     // DEBUG_PRINT("attitude_rate: %f; %f; %f\n", (double)stateVect[9][0], (double)stateVect[10][0], (double)stateVect[11][0]);

    //     DEBUG_PRINT("setpoint: %f; %f; %f\n", (double)setpoint->position.x, (double)setpoint->position.y, (double)setpoint->position.z);

    //     // DEBUG_PRINT("control: %f, %f, %f, %f\n", (double)(controlVect[0][0]), (double)(controlVect[1][0]), (double)(controlVect[2][0]), (double)(controlVect[3][0]));
        
    //     i = 0;
    // }
    // else{
    //     i++;
    // }




}


LOG_GROUP_START(error_xyz)
/**
 * @brief x error
 */
LOG_ADD(LOG_FLOAT, err_x, &(controlState[0]))
/**
 * @brief y error
 */
LOG_ADD(LOG_FLOAT, err_y, &(controlState[1]))
/**
 * @brief z error
 */
LOG_ADD(LOG_FLOAT, err_z, &(controlState[2]))

LOG_GROUP_STOP(error_xyz)



LOG_GROUP_START(state_control)
/**
 * @brief x pose of the drone
 */
LOG_ADD(LOG_FLOAT, x, &(stateVect[0]))
/**
 * @brief y pose of the drone
 */
LOG_ADD(LOG_FLOAT, y, &(stateVect[1]))
/**
 * @brief z pose of the drone
 */
LOG_ADD(LOG_FLOAT, z, &(stateVect[2]))
/**
 * @brief vx velocity of the drone
 */
LOG_ADD(LOG_FLOAT, vx, &(stateVect[3]))
/**
 * @brief vy velocity of the drone
 */
LOG_ADD(LOG_FLOAT, vy, &(stateVect[4]))
/**
 * @brief vz velocity of the drone
 */
LOG_ADD(LOG_FLOAT, vz, &(stateVect[5]))
/**
 * @brief roll angle of the drone
 */
LOG_ADD(LOG_FLOAT, roll, &(stateVect[6]))
/**
 * @brief pitch angle of the drone
 */
LOG_ADD(LOG_FLOAT, pitch, &(stateVect[7]))
/**
 * @brief yaw angle of the drone
 */
LOG_ADD(LOG_FLOAT, yaw, &(stateVect[8]))
/**
 * @brief wx angle velocity of the drone
 */
LOG_ADD(LOG_FLOAT, wx, &(stateVect[9]))
/**
 * @brief wy angle velocity pose of the drone
 */
LOG_ADD(LOG_FLOAT, wy, &(stateVect[10]))
/**
 * @brief wz angle velocity pose of the drone
 */
LOG_ADD(LOG_FLOAT, wz, &(stateVect[11]))
/**
 * @brief thrust on m1
 */
LOG_ADD(LOG_FLOAT, m1, &(controlVect[0]))
/**
 * @brief thrust on m2
 */
LOG_ADD(LOG_FLOAT, m2, &(controlVect[1]))
/**
 * @brief thrust on m3
 */
LOG_ADD(LOG_FLOAT, m3, &(controlVect[2]))
/**
 * @brief thrust on m4
 */
LOG_ADD(LOG_FLOAT, m4, &(controlVect[3]))

LOG_GROUP_STOP(state_control)




LOG_GROUP_START(target_state)

/**
 *  @brief target x pose
 */
LOG_ADD(LOG_FLOAT, x, &(refVect[0]))
/**
 *  @brief target y pose
 */
LOG_ADD(LOG_FLOAT, y, &(refVect[1]))
/**
 *  @brief target z pose
 */
LOG_ADD(LOG_FLOAT, z, &(refVect[2]))
/**
 *  @brief target yaw angle
 */
LOG_ADD(LOG_FLOAT, yaw, &(refVect[8]))

LOG_GROUP_STOP(target_state)
