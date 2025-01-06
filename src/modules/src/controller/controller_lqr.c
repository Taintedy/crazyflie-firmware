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
// float K[4][12] = {
//     {-0.01118034, 0.01118034, 0.11180340, -0.00658162, 0.00658162, 0.04683335, -0.01900418, -0.01900418, -0.07905694, -0.00310681, -0.00310681, -0.01468404},
//     {0.01118034, 0.01118034, 0.11180340, 0.00846931, 0.00846931, 0.04683335, -0.02050084, 0.02050084, 0.07905694, -0.00277769, 0.00277769, 0.01468404},
//     {0.01118034, -0.01118034, 0.11180340, 0.00658162, -0.00658162, 0.04683335, 0.01900418, 0.01900418, -0.07905694, 0.00310681, 0.00310681, -0.01468404},
//     {-0.01118034, -0.01118034, 0.11180340, -0.00846931, -0.00846931, 0.04683335, 0.02050084, -0.02050084, 0.07905694, 0.00277769, -0.00277769, 0.01468404}
// };

float K[4][12] = {
    {-0.01118034, 0.01118034, 0.11180340, -0.00657166, 0.00657166, 0.04683335, -0.01894670, -0.01894670, -0.07905694, -0.00310333, -0.00310333, -0.01468404},
    {0.01118034, 0.01118034, 0.11180340, 0.00846931, 0.00846931, 0.04683335, -0.02050084, 0.02050084, 0.07905694, -0.00277769, 0.00277769, 0.01468404},
    {0.01118034, -0.01118034, 0.11180340, 0.00657166, -0.00657166, 0.04683335, 0.01894670, 0.01894670, -0.07905694, 0.00310333, 0.00310333, -0.01468404},
    {-0.01118034, -0.01118034, 0.11180340, -0.00846931, -0.00846931, 0.04683335, 0.02050084, -0.02050084, 0.07905694, 0.00277769, -0.00277769, 0.01468404}
};


bool isInit = false;


void controllerLQRInit(void)
{   
    isInit = true;
}

bool controllerLQRTest(void)
{
    return isInit;
}


int max_time = 100;
int i = 0;
void controllerLQR(control_t *control, const setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const stabilizerStep_t stabilizerStep)
{

    control->controlMode = controlModeForce;
    //  && setpoint->mode.z != modeDisable

    if (RATE_DO_EXECUTE(RATE_MAIN_LOOP, stabilizerStep) && setpoint->mode.z != modeDisable)
    {
        createStateVector(&stateVect, state, sensors);
        createRefVector(&refVect, setpoint);
        subtractMatrices(12, 1, &refVect, &stateVect, &controlState);

        // controlState[0][0] = 0.0f;
        // controlState[1][0] = 0.0f;

        matrixMultiply(4, 12, 1, &K, &controlState, &controlVect);

        for(int i = 0; i < 4; i ++)
        {
            controlVect[i][0] += F_HOVER;
        }

        clipMatrix(4, 1, 0.0f, F_MAX, &controlVect);
        // normalize(4, 1, F_MAX, &controlVect);

        control->normalizedForces[0] = controlVect[0][0];
        control->normalizedForces[1] = controlVect[1][0];
        control->normalizedForces[2] = controlVect[2][0];
        control->normalizedForces[3] = controlVect[3][0];


        
    }

    // if(i == max_time)
    // {
    //     // DEBUG_PRINT("pose: %f; %f; %f\n", (double)stateVect[0][0], (double)stateVect[1][0], (double)stateVect[2][0]);
    //     // DEBUG_PRINT("actual pose: %f; %f; %f\n", (double)state->position.x, (double)state->position.y, (double)state->position.z);

    //     // DEBUG_PRINT("vel: %f; %f; %f\n", (double)stateVect[3][0], (double)stateVect[4][0], (double)stateVect[5][0]);
    //     // DEBUG_PRINT("attitude: %f; %f; %f\n", (double)stateVect[6][0], (double)stateVect[7][0], (double)stateVect[8][0]);
    //     DEBUG_PRINT("attitude_rate: %f; %f; %f\n", (double)stateVect[9][0], (double)stateVect[10][0], (double)stateVect[11][0]);

    //     // DEBUG_PRINT("setpoint: %f; %f; %f\n", (double)setpoint->position.x, (double)setpoint->position.y, (double)setpoint->position.z);

    //     // DEBUG_PRINT("control: %f, %f, %f, %f\n", (double)(controlVect[0][0]), (double)(controlVect[1][0]), (double)(controlVect[2][0]), (double)(controlVect[3][0]));
        
    //     i = 0;
    // }
    // else{
    //     i++;
    // }




}


