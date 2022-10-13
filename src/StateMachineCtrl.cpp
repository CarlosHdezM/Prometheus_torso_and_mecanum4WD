/*
 Inbiodroid MCU UDP socket teleoperation state machine:
 This program receives UDP datagrams, containing either SM (state machine) commands
 or (position) control input data, and acts accordingly based upon current state and
 of course, the nature of the module to be controlled. This implementation file focuses
 on the code executed for every command received.

 created 23 Sep 2022
 by Inbiodroid technical team.

 This code is property of Inbiodroid.
 */

//#include <atomic>  // do you need to make your setpoints variables atomic?
#include "StateMachine.h"
#include "PrometheusTeleoperation-MotorsSetup.h"
#include "mecanum_4wd.h"



// IMPORTANT: make sure to set the is_outbound_data_ready flag to 'true' everytime the control
// interruption is run; we do not send the tx buffer regardless, lest we flood the network
// unnecesaryly.
//
//


/**
 * @brief This global variable is meant to save the current state.
 */
State current_state = State::DISCONNECTED;
bool is_outbound_data_ready = false;



void periodicInterruptCallback()
{
    //-----Torso
    controlMotors();

    //-----Mecanum4WD
    mecanum4WD_updateVelControl(T);
}



/**
 * @brief DISCONNECTED->CONNECTED transition. This function should implement
 * such transition by validating the UDP communications.
 * 
 * @return uint8_t StateMachineError code. 
 */
uint8_t connect()
{
    //Torso: No action needed
    //Mecanum4WD: No action needed
    return (uint8_t)StateMachineError::TRANSITION_OK;
}



/**
 * @brief CONNECTED->VALIDATED transition.This function is intended to validate
 * the embedded systems and actuators (motors).
 * .
 * 
 * @return uint8_t StateMachineError code.
 */
uint8_t validate()
{
    //-----Torso:
    motors[0]->setTorque(0.0);
    motors[1]->setTorque(0.0);
    motors[1]->setTorque(0.0);

    for (auto &motor : motors) // Implement a counter to get the loop number
    {
        //ToDo: Replace the (possibly) inifinite while loop with a timeout. 
        while (!motor->initialize())
        {
            Debug("Retrying to initialize ");
            Serial.print(motor->name());
            Serial.print(" MCP2515");
        }
        //ToDo: Replace the (possibly) inifinite while loop with a timeout. 
        while (!motor->turnOn())
        {
            Debug("Retrying to turn on ");
            Serial.println(motor->name());
        }
    }
    Debug("All motors initialized succesfully");

    //-----Mecanum4WD
    ErrorMecanum4WD mecanum_status = mecanum4WD_initialize();

    //ToDo: Return an appropriate StateMachineError rather than assuming TRANSITION_OK.

    return (uint8_t)StateMachineError::TRANSITION_OK;
}



/**
 * @brief VALIDATED->CALIBRATED transition. This function should perform the
 * required actions to calibrate the module controlled.
 * 
 * @return uint8_t StateMachineError code.
 */
uint8_t calibrate()
{
    //-----Torso
    //Set the zero position reference of the torso motors
    for (uint8_t i = 0; i < NUM_MOTORS; i++)
    {
        Serial.print(motors[i]->name());
        if (motors[i]->setCurrentPositionAsZero())
        {
            Serial.print("Position: ");
            Serial.print(motors[i]->position(), 4);
            Serial.print("\tTorque: ");
            Serial.print(motors[i]->torque(), 4);
            Serial.print("\tVelocity: ");
            Serial.println(motors[i]->velocity(), 4);
        }
        else
        {
            Serial.println(":\tFailed setting current position as origin");
        }
    }
    //ToDo: Check if the position of each motor is in fact zero or near to zero. 


    //-----Mecanum4WD
    //Send velocity zero to mecanum4WD robot.  
    ErrorMecanum4WD is_mecanum4WD_stopped = mecanum4WD_stopRobotBlocking(2000);


    //-----Torso
    //Enable torso motors auto mode. 
    for (uint8_t i = 0; i < NUM_MOTORS; i++) // ESTO SE DEBE DE HACER UNA SOLA VEZ
    {
        Serial.print("Starting auto mode for:");
        Serial.println(motors[i]->name());
        motors[i]->startAutoMode(interrupt_handlers[i]);
    }

    //-----Mecanum4WD
    //Enable mecanum4WD motors auto mode.
    mecanum4WD_velocity_setpoint.setVelocities(0.0, 0.0, 0.0);
    mecanum4WD_enableMotorsAutoMode();

    //-----Torso and Mecanum4WD
    //Enable periodic interrupt.
    myTimer.begin(periodicInterruptCallback, PERIOD_USEC);
    myTimer.priority(200);

    return (uint8_t)StateMachineError::TRANSITION_OK;
}



/**
 * @brief CALIBRATED->HOME transition. This function should perform the control
 * required to reach the home secure position from which engage in teleoperation
 * is possible.
 * 
 * @return uint8_t StateMachineError code.
 */
uint8_t gotohome()
{
    //System's periodic interrupt (that updates Torso and Mecanum4wd control law) was enabled in calibrate() and should be still active.
    //Set control law setpoint.

    //-----Torso
    Beta = 0.0;
    Gamma = 0.0;
        
    //-----Mecanum4WD
    mecanum4WD_velocity_setpoint.setVelocities(0.0, 0.0, 0.0);

    return (uint8_t)StateMachineError::TRANSITION_OK;
}



/**
 * @brief HOME->ENGAGED transition. Ensure here that the transition to engage
 * is clean and safe. After doing this, the data processing/consumption is started.
 * 
 * @return uint8_t StateMachineError code.
 */
uint8_t engage()
{
    //System's periodic interrupt (that updates Torso and Mecanum4wd control law) was enabled in calibrate() and should be still active.
    //Setpoints get loaded in updateIncomingData(). The state machine calls it when new data is received while engaged. 
    //-----Torso: No further action needed.
    //-----Mecanum4WD: No further action needed.
    return (uint8_t)StateMachineError::TRANSITION_OK;
}



/**
 * @brief ENGAGED->DISENGAGED transition. This function stops the data consumption
 * and halts the robot.
 * 
 * @return uint8_t StateMachineError code.
 */
uint8_t disengage()
{
    //System's periodic interrupt (that updates Torso and Mecanum4wd control law) was enabled in calibrate() and should be still active.
    //Set control law setpoint.

    //-----Torso
    Beta = 0.0;
    Gamma = 0.0;

    //-----Mecanum4WD
    mecanum4WD_velocity_setpoint.setVelocities(0.0, 0.0, 0.0);
    return (uint8_t)StateMachineError::TRANSITION_OK;
}



/**
 * @brief DISENGAGED->HOME transition. This function should be performed whenever
 * coming from the disengaged state to go back to home.
 * 
 * @return uint8_t StateMachineError code.
 */
uint8_t gobacktohome()
{
    //System's periodic interrupt (that updates Torso and Mecanum4wd control law) was enabled in calibrate() and should be still active.
    //Set control law setpoint.
    //-----Torso
    Beta = 0.0;
    Gamma = 0.0;

    //-----Mecanum4WD
    mecanum4WD_velocity_setpoint.setVelocities(0.0, 0.0, 0.0);
    return (uint8_t)StateMachineError::TRANSITION_OK;
}



/**
 * @brief HOME->DISCONNECTED transition. This function is to finish exit the escenario.
 * 
 * @return uint8_t StateMachineError code.
 */
uint8_t disconnect()
{
    //-----Torso
    Debug("\nDisabling Motor");
    for (uint8_t i = 0; i < NUM_MOTORS; i++)
    {
        if (motors[i]->turnOff())
        {
            Serial.print("Turned off ");
            Serial.println(motors[i]->name());
        }
        else
        {
            Debug("Failed turning off ");
            Debugln(motors[i]->name());
        }
    }

    //-----Mecanum4WD
    mecanum4WD_turnOff();

    //-----Torso and Mecanum4WD
    //Disable periodic interrupt.
    myTimer.end();

    return (uint8_t)StateMachineError::TRANSITION_OK;
}



/**
 * @brief DISCONNECTED->POWEREDOFF transition. This function power down the robot.
 * 
 * @return uint8_t StateMachineError code.
 */
uint8_t poweroff()
{
    return (uint8_t)StateMachineError::TRANSITION_OK;
}



/**
 * @brief This function is intended to update the input data coming into the rx buffer
 * 
 * @return unint8_t 
 */
void updateIncomingData()
{
    //Torso
    Beta = p2pComm._rxDatagram.payload.Q[0];
    Gamma = p2pComm._rxDatagram.payload.Q[1];

    //Mecanum4WD
    mecanum4WD_velocity_setpoint.setVelocities(
        p2pComm._rxDatagram.payload.Q[2],
        p2pComm._rxDatagram.payload.Q[3],
        p2pComm._rxDatagram.payload.Q[4] );

    return;
}



/**
 * @brief This function is intended to update the output data on the tx buffer.
 * This data is subsequently picked up and sent on the next StateMachine loop.
 * 
 */
void updateOutgoingData()
{
    /* code here to consume data, i.e. put outgoing data in the tx buffer*/
    is_outbound_data_ready = false; // clear the flag; set it in the interrupt callback
    return;
}