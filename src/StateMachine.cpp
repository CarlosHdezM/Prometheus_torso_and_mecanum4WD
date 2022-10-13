/*
 Inbiodroid MCU UDP socket teleoperation state machine:
 This program receives UDP datagrams, containing either SM (state machine) commands
 or (position) control input data, and acts accordingly based upon current state and
 of course, the nature of the module to be controlled.

 created 23 Sep 2022
 by Inbiodroid technical team.

 This code is property of Inbiodroid.
 */

#include <tuple>
#include "StateMachine.h"

//Messaging objects
#ifdef AVTR_NECK
P2pSocket<headsetOptrPayload,     neckAvtrPayload>        p2pComm(neckAvtrMcuEpConfig); // marteaga: you MUST tell the linker that these types are allowed to be instantiated in the class
#elif OPTR_LEFT_EXO
P2pSocket<armAvtrPayload,         exoOptrPayload>         p2pComm(leftExoOptrMcuEpConfig); // exo MCU
#elif AVTR_LEFT_ARM
P2pSocket<exoOptrPayload,         armAvtrPayload>         p2pComm(leftArmAvtrMcuEpConfig); // arm MCU
#elif OPTR_RIGHT_EXO
P2pSocket<armAvtrPayload,         exoOptrPayload>         p2pComm(rightExoOptrMcuEpConfig);
#elif AVTR_RIGHT_ARM
P2pSocket<exoOptrPayload,         armAvtrPayload>         p2pComm(rightArmAvtrMcuEpConfig);
#elif OPTR_LEFT_3DOF_GLOVE
P2pSocket<hand3DofAvtrPayload,    glove3DofOptrPayload>   p2pComm(left3DoFGloveMcuEpConfig);
#elif AVTR_LEFT_HAND
P2pSocket<glove3DofOptrPayload,   hand3DofAvtrPayload>    p2pComm(leftHandMcuEpConfig);
#elif OPTR_RIGHT_R5F_GLOVE
P2pSocket<hand5fAvtrPayload,      glove5fOptrPayload>     p2pComm(right5FGloveMcuEpConfig);
#elif AVTR_RIGHT_HAND
P2pSocket<glove5fOptrPayload,     hand5fAvtrPayload>      p2pComm(rightHandMcuEpConfig);
#elif OPTR_TOUCH_SENSE
P2pSocket<touchSenseAvtrPayload,  touchSenseOptrPayload>  p2pComm(optrTouchSenseMcuEpConfig);
#elif AVTR_TOUCH_SENSE
P2pSocket<touchSenseOptrPayload,  touchSenseAvtrPayload>  p2pComm(avtrTouchSenseMcuEpConfig);
#elif OPTR_PEDALIMU
P2pSocket<txOmniwheelsPayload,    rxOmniwheelsPayload>    p2pComm(optrPedalImuMcuEpConfig);
#elif AVTR_OMNIWHEELS
P2pSocket<rxOmniwheelsPayload,    txOmniwheelsPayload>    p2pComm(avtrOmniwheelsMcuEpConfig);
#elif AVTR_OMNIWHEELS_BKUP
P2pSocket<rxOmniwheelsPayload,    txOmniwheelsPayload>    p2pComm(avtrOmniwheelsBkupMcuEpConfig);
#elif OPTR_TEST
P2pSocket<testAvtrPayload,        testOptrPayload>        p2pComm(operatorsideTestConfig);
#elif AVTR_TEST
P2pSocket<testOptrPayload,        testAvtrPayload>        p2pComm(avatarsideTestConfig);
#elif ASIDE_PAUL_TEST
P2pSocket<testAvtrPayload,        testOptrPayload>        p2pComm(aSidePaulTestConfig);
#elif BSIDE_PAUL_TEST
P2pSocket<testAvtrPayload,        testOptrPayload>        p2pComm(bSidePaulTestConfig);
#else
P2pSocket<testAvtrPayload,        testOptrPayload>        p2pComm(operatorsideTestConfig);
#endif


void setup()
{
  // Open serial communications and wait for port to open.
  #ifdef DEBUG_LOG
    Serial.begin(9600);
    while (!Serial) {
      ; // wait for serial port to connect. Needed for native USB port only.
    }
  #endif
  p2pComm.begin(); // Ethernet start; the cofiguration come from the McuConfig object.
}


void loop()
{
  static UdpError error       = UdpError::UNKNOWN;
  static Command command      = Command::UNKNOWN;
  //p2pComm._txDatagram.updateCrc();

  /* 1. Check if a new datagram is available. */
  std::tie(error, command) = p2pComm.receiveDatagram();

  switch (error)
  {
  case UdpError::MESSAGE_OK:
  /* 2. A new UDP message has been received & validated. Execute command then. */
    switch (command)
    {

    case Command::ACK:
    /* 3. For every command, evaluate whether you can excute it based on the current state. */
      switch (current_state)
      {
        case State::DISCONNECTED:
          static uint64_t outgoing_counter = 0; //counter to limit the amount of output to the serial
          // fill the payload with printable chars
          for(unsigned int i = 0; i < sizeof(p2pComm._txDatagram.payload); i++)
          {
            p2pComm.getTxPayloadBuffer()[i] = byte(' '+((outgoing_counter+i)%95));
          }
          p2pComm._txDatagram.returnCode = (uint8_t)outgoing_counter++; // assign the variable and increment 
          // p2pComm._txDatagram.timecode.m_hours    = 0x40;
          // p2pComm._txDatagram.timecode.m_minutes  = 0x41;
          // p2pComm._txDatagram.timecode.m_seconds  = 0x42;
          // p2pComm._txDatagram.timecode.m_frame    = 0x4443;
          //p2pComm._txDatagram.updateCrc(); // update CRC last; is it updated right before sending every packet
          p2pComm.sendDatagram(MessageType::STATEMACHINE_ACK);
        break;
        default:
          #ifdef DEBUG_LOG
          Debug("Estado actual inválido para atender este comando: ");
          Serial.printf("<0x%02,0x%02,0x%02>",error,command,current_state);
          #endif
        break;
      }
    break;


    case Command::SM_CONNECT:
      switch (current_state)
      {
      case State::DISCONNECTED:
        Debugln("Try DISCONNECTED->CONNECTED");
        p2pComm._txDatagram.returnCode = connect();
        current_state = p2pComm._txDatagram.returnCode == 0 ? State::CONNECTED : State::DISCONNECTED;
      break;

      case State::CONNECTED:
        Debugln("Already CONNECTED.");
        p2pComm._txDatagram.returnCode = (uint8_t)StateMachineError::TRANSITION_DONE;
      break;

      default:
        #ifdef DEBUG_LOG
        Serial.printf("Invalid current state 0x%02X for received command 0x%02X.\n", current_state, command);
        #endif
        p2pComm._txDatagram.returnCode = (uint8_t)StateMachineError::TRANSITION_INVALID;
      break;
      }
      p2pComm.sendDatagram(MessageType::STATEMACHINE_ACK);
    break;


    case Command::SM_VALIDATE:
      switch (current_state)
      {
      case State::CONNECTED:
        Debugln("Try CONNECTED->VALIDATED");
        p2pComm._txDatagram.returnCode = validate();
        current_state = p2pComm._txDatagram.returnCode == 0 ? State::VALIDATED : State::CONNECTED;
      break;
      
      case State::VALIDATED:
        Debugln("Already VALIDATED");
        p2pComm._txDatagram.returnCode = (uint8_t)StateMachineError::TRANSITION_DONE;
      break;
      
      default:
        #ifdef DEBUG_LOG
        Serial.printf("Invalid current state 0x%02X for received command 0x%02X.\n", current_state, command);
        #endif
        p2pComm._txDatagram.returnCode = (uint8_t)StateMachineError::TRANSITION_INVALID;
      break;
      }
      p2pComm.sendDatagram(MessageType::STATEMACHINE_ACK);
    break;


    case Command::SM_CALIBRATE:
      switch (current_state)
      {
      case State::VALIDATED:
        Debugln("Try VALIDATED->CALIBRATED");
        p2pComm._txDatagram.returnCode = calibrate();
        current_state = p2pComm._txDatagram.returnCode == 0 ? State::CALIBRATED : State::VALIDATED;
      break;
      case State::CALIBRATED:
        Debugln("Already CALIBRATED");
        p2pComm._txDatagram.returnCode = (uint8_t)StateMachineError::TRANSITION_DONE;
      break;
      default:
        #ifdef DEBUG_LOG
        Serial.printf("Invalid current state 0x%02X for received command 0x%02X.\n", current_state, command);
        #endif
        p2pComm._txDatagram.returnCode = (uint8_t)StateMachineError::TRANSITION_INVALID;
      break;
      }
      p2pComm.sendDatagram(MessageType::STATEMACHINE_ACK);
    break;


    case Command::SM_GOTOHOME:
      switch (current_state)
      {
      case State::CALIBRATED:
        Debugln("Try CALIBRATED->HOME");
        p2pComm._txDatagram.returnCode = gotohome();
        current_state = p2pComm._txDatagram.returnCode == 0 ? State::HOME : State::CALIBRATED;
      break;
      case State::DISENGAGED:
        Debugln("Try DISENGAGED->HOME");
        p2pComm._txDatagram.returnCode = gobacktohome();
        current_state = p2pComm._txDatagram.returnCode == 0 ? State::HOME : State::DISENGAGED;
      break;
      case State::HOME:
        Debugln("Already at HOME");
        p2pComm._txDatagram.returnCode = (uint8_t)StateMachineError::TRANSITION_DONE;
      break;
      default:
        #ifdef DEBUG_LOG
        Serial.printf("Invalid current state 0x%02X for received command 0x%02X.\n", current_state, command);
        #endif
        p2pComm._txDatagram.returnCode = (uint8_t)StateMachineError::TRANSITION_INVALID;
      break;
      }
      p2pComm.sendDatagram(MessageType::STATEMACHINE_ACK);
    break;


    case Command::SM_ENGAGE:
      switch (current_state)
      {
      case State::HOME:
        Debugln("Try HOME->ENGAGED");
        p2pComm._txDatagram.returnCode = engage();
        current_state = p2pComm._txDatagram.returnCode == 0 ? State::ENGAGED : State::HOME;
      break;
      case State::ENGAGED:
        Debugln("Already ENGAGED");
        p2pComm._txDatagram.returnCode = (uint8_t)StateMachineError::TRANSITION_DONE;
      break;
      default:
        #ifdef DEBUG_LOG
        Serial.printf("Invalid current state 0x%02X for received command 0x%02X.\n", current_state, command);
        #endif
        p2pComm._txDatagram.returnCode = (uint8_t)StateMachineError::TRANSITION_INVALID;
      break;
      }
      p2pComm.sendDatagram(MessageType::STATEMACHINE_ACK);
    break;


    case Command::SM_DISENGAGE:
      switch (current_state)
      {
      case State::ENGAGED:
        Debugln("Try ENGAGED->DISENGAGED");
        p2pComm._txDatagram.returnCode = disengage();
        current_state = p2pComm._txDatagram.returnCode == 0 ? State::DISENGAGED : State::ENGAGED;
      break;
      case State::DISENGAGED:
        Debugln("Already on DISENGAGED");
        p2pComm._txDatagram.returnCode = (uint8_t)StateMachineError::TRANSITION_DONE;
      break;
      default:
        #ifdef DEBUG_LOG
        Serial.printf("Invalid current state 0x%02X for received command 0x%02X.\n", current_state, command);
        #endif
        p2pComm._txDatagram.returnCode = (uint8_t)StateMachineError::TRANSITION_INVALID;
      break;
      }
      p2pComm.sendDatagram(MessageType::STATEMACHINE_ACK);
    break;


    case Command::SM_DISCONNECT:
      switch (current_state)
      {
      case State::HOME:
        Debugln("Try HOME->DISCONNECTED");
        p2pComm._txDatagram.returnCode = disconnect();
        current_state = p2pComm._txDatagram.returnCode == 0 ? State::DISCONNECTED : State::HOME;
      break;
      case State::DISCONNECTED:
        Debugln("Already on DISCONNECTED");
        p2pComm._txDatagram.returnCode = (uint8_t)StateMachineError::TRANSITION_DONE;
      break;
      // case State::ENGAGED:
      //   Debugln("Try ENGAGED->DISCONNECTED");
      //   p2pComm._txDatagram.returnCode = disconnect();
      //   current_state = State::DISCONNECTED;
      // break;
      default:
        #ifdef DEBUG_LOG
        Serial.printf("Invalid current state 0x%02X for received command 0x%02X.\n", current_state, command);
        #endif
        p2pComm._txDatagram.returnCode = (uint8_t)StateMachineError::TRANSITION_INVALID;
      break;
      }
      p2pComm.sendDatagram(MessageType::STATEMACHINE_ACK);
    break;


    case Command::SM_POWEROFF:
      switch (current_state)
      {
      case State::DISCONNECTED:
        Debugln("Try DISCONNECTED->POWEREDOFF");
        p2pComm._txDatagram.returnCode = poweroff();
        current_state = p2pComm._txDatagram.returnCode == 0 ? State::POWEREDOFF : State::DISCONNECTED;
        break;
      
      default:
        #ifdef DEBUG_LOG
        Serial.printf("Invalid current state 0x%02X for received command 0x%02X.\n", current_state, command);
        #endif
        p2pComm._txDatagram.returnCode = (uint8_t)StateMachineError::TRANSITION_INVALID;
        break;
      } 
      p2pComm.sendDatagram(MessageType::STATEMACHINE_ACK);
    break;


    case Command::TO_RECEIVEDATA:
    /* 3.1 If the command is to consume new data, do something here? */
      switch (current_state)
      {
      case State::ENGAGED:
        // Debugln("Teleoperation data update.");
        updateIncomingData();
      break;
      default:
        #ifdef DEBUG_LOG
        //Serial.printf("Invalid current state 0x%02X for received command 0x%02X.\n", current_state, command);
        #endif
        p2pComm._txDatagram.returnCode = (uint8_t)StateMachineError::TRANSITION_INVALID;
      break;
      }
    break;

    default:
    /* 3.2 ToDo: handle unknown command. */
    break;
    }
  break;


  case UdpError::NO_PACKET:
  /* 2.1 No new UDP; do something here? Yes! Maybe. Maybe, check here if we have output data to be sent... */
  break;

  default:
  /* 2.3 ToDo: Handle UDP error. In the meantime, just print the specific error.  */
    #ifdef DEBUG_LOG
      Serial.printf("<UdpError::0x%02X>\n",error);
    #endif
    p2pComm._rxDatagram.command = (uint8_t)command;
    p2pComm._rxDatagram.returnCode = (uint8_t)error;
    //p2pComm._txDatagram.updateCrc(); // update CRC last
    p2pComm.sendDatagram(MessageType::STATEMACHINE_ACK);
  break;
  }



  /* 4. Now, based on current state, do something additional? */
  switch (current_state)
  {
  // ... or maybe here, everytime the loop ends, check if ENGAGED and send ouput data if ready to go.
  case State::ENGAGED:
  /* 4.1 Send new output data if it's ready to go. */
    if (is_outbound_data_ready)  // flag set by the interrupt
    {
      updateOutgoingData();
      p2pComm._txDatagram.returnCode = (uint8_t)StateMachineError::TRANSITION_OK;
      p2pComm.sendDatagram(MessageType::TELEOPERATION_DATA);
    }
    
    // send new positions to peer.
  break;
  
  default:
  break;
  }

  #if 0 // delays for debugging purposed; enable at your own peril!
  delay(1000); // 1Hz
  #elif 0
  delayMicroseconds(250); // 4KHz
  #endif
}

/*
// Processing UDP example to send and receive data buffer;
// press any key to send the byte message
import hypermedia.net.*;

UDP udp;

String ip = "16.221.3.122";                                // the remote IP address (avatarsideMCUTestConfig)
int port = 0xC07A;                                         // the destination port (avatarsideMCUTestConfig)
int buffer_size = 8*4;                                     // Max buffer size on the NativeEthernet.h es is by default (Teensy4.1/PlatformIO)
byte[] phis = new byte[buffer_size];                       // 7 32-bit floats for arm teleoperation

void setup() {
  udp = new UDP(this, 8817);                               // create a new datagram connection on port (RX)
  udp.log(true);                                           // the connection activity
  udp.listen(true);                                        // and wait for incoming message
}

void draw(){
}

void keyPressed() {
  for(int i = 0; i < buffer_size; i++){
    phis[i] = byte(i+'A');
  }
  udp.send(phis, ip, port );                               // send the message
}

void receive(byte[] data) {                                // <-- default handler
//void receive(byte[] data, String ip, int port ) {        // <-- extended handler
  for(int i=0; i < data.length; i++)
    print(char(data[i]));
  println();
}
*/