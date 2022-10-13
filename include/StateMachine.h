#ifndef STATEMACHINE_H
    #define STATEMACHINE_H
    #include "P2pSocket.h"

    enum class State
    {
        DISCONNECTED,
        CONNECTED,
        VALIDATED,
        CALIBRATED,
        HOME,
        ENGAGED,
        DISENGAGED,
        POWEREDOFF
    };

    /**
     * @brief Types of error sent in the returnCode field.
     * 
     */
    enum class StateMachineError
    {
        TRANSITION_OK = 0,
        TRANSITION_DONE,
        TRANSITION_INVALID,
        TRANSITION_ERROR = 0xFF
    };

    extern State    current_state;
    extern uint8_t  connect();
    extern uint8_t  validate();
    extern uint8_t  calibrate();
    extern uint8_t  gotohome();
    extern uint8_t  gobacktohome();
    extern uint8_t  engage();
    extern uint8_t  disengage();
    extern uint8_t  disconnect();
    extern uint8_t  poweroff();

    extern    void  updateIncomingData();
    extern    void  updateOutgoingData();
    extern    bool  is_outbound_data_ready;


    //Messaging objects
    #ifdef AVTR_NECK
    extern P2pSocket<headsetOptrPayload,     neckAvtrPayload>        p2pComm; // marteaga: you MUST tell the linker that these types are allowed to be instantiated in the class
    #elif OPTR_LEFT_EXO
    extern P2pSocket<armAvtrPayload,         exoOptrPayload>         p2pComm; // exo MCU
    #elif AVTR_LEFT_ARM
    extern P2pSocket<exoOptrPayload,         armAvtrPayload>         p2pComm; // arm MCU
    #elif OPTR_RIGHT_EXO
    extern P2pSocket<armAvtrPayload,         exoOptrPayload>         p2pComm;
    #elif AVTR_RIGHT_ARM
    extern P2pSocket<exoOptrPayload,         armAvtrPayload>         p2pComm;
    #elif OPTR_LEFT_3DOF_GLOVE
    extern P2pSocket<hand3DofAvtrPayload,    glove3DofOptrPayload>   p2pComm;
    #elif AVTR_LEFT_HAND
    extern P2pSocket<glove3DofOptrPayload,   hand3DofAvtrPayload>    p2pComm;
    #elif OPTR_RIGHT_R5F_GLOVE
    extern P2pSocket<hand5fAvtrPayload,      glove5fOptrPayload>     p2pComm;
    #elif AVTR_RIGHT_HAND
    extern P2pSocket<glove5fOptrPayload,     hand5fAvtrPayload>      p2pComm;
    #elif OPTR_TOUCH_SENSE
    extern P2pSocket<touchSenseAvtrPayload,  touchSenseOptrPayload>  p2pComm;
    #elif AVTR_TOUCH_SENSE
    extern P2pSocket<touchSenseOptrPayload,  touchSenseAvtrPayload>  p2pComm;
    #elif OPTR_PEDALIMU
    extern P2pSocket<txOmniwheelsPayload,    rxOmniwheelsPayload>    p2pComm;
    #elif AVTR_OMNIWHEELS
    extern P2pSocket<rxOmniwheelsPayload,    txOmniwheelsPayload>    p2pComm;
    #elif AVTR_OMNIWHEELS_BKUP
    extern P2pSocket<rxOmniwheelsPayload,    txOmniwheelsPayload>    p2pComm;
    #elif OPTR_TEST
    extern P2pSocket<testAvtrPayload,        testOptrPayload>        p2pComm;
    #elif AVTR_TEST
    extern P2pSocket<testOptrPayload,        testAvtrPayload>        p2pComm;
    #elif ASIDE_PAUL_TEST
    extern P2pSocket<testAvtrPayload,        testOptrPayload>        p2pComm;
    #elif BSIDE_PAUL_TEST
    extern P2pSocket<testAvtrPayload,        testOptrPayload>        p2pComm;
    #else
    extern P2pSocket<testAvtrPayload,        testOptrPayload>        p2pComm;
    #endif

#endif