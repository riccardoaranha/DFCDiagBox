#ifndef __DIAGController__
#define __DIAGController__

#include <Arduino.h>
#include <ACAN2517FD.h>
#include <SPI.h>
#include <StateMachine.h> 
#include <ArduinoQueue.h> 

typedef struct request {
    long size;
    uint8_t *data;
} DIAGRequest;

 // PIN and Board definition
static const byte MCP2517_CS  = 17 ; // CS input of MCP2517
static const byte MCP2517_INT =  7 ; // INT output of MCP2517
static const ACAN2517FDSettings::Oscillator mOscillator = ACAN2517FDSettings::OSC_20MHz; //Board Oscillator
// Max size of the queue of messages
static const int QUEUE_MAX_SIZE = 32;


        
// Tester Present message
static uint8_t TesterPresent[2] = { 0x3E, 0x00 };

class DIAGController {
    public: 
        
        // Variable to check if the overall is successfull
        static bool OverallSuccess;
        static bool DiagResponseReceived;
        static bool DiagLastResponsePositive;


        // Check if the CAN Controller is Idle;
        static bool isIdle();

        static int addMessage(uint8_t data[], int data_size);
        static int startRoutine();



        static int setup(uint32_t inArbitrationBitRate, //Can Baud Rate. e.g. 500UL * 1000UL = 500kpbs
                         DataBitRateFactor inDataBitRateFactor, // Multiplier for Data baud rate 
                         uint32_t inFrameID_Request, // ID of the Diag Request frame
                         uint32_t inFrameID_Response, // ID of the Diag Response frame
                         uint32_t inTesterPresent_Timer = 3000, // Periodicity to send the Tester Present. 0 to don't send
                         uint32_t inDiagTimeout = 5000); // Timeout to wait for a Diag response);
        static void loop();

    private:
        // Control Variables
        static uint32_t mArbitrationBitRate;
        static DataBitRateFactor mDataBitRateFactor;
        static uint32_t mFrameID_Request;
        static uint32_t mFrameID_Response;
        static uint32_t mTesterPresent_Timer;
        static uint32_t mDiagTimeout;
        static uint32_t gSendTesterPresentDate;
        static uint8_t* lastDiagRequest;
        static uint32_t inDiagRequestSentTime;
        static bool bDiagResponseReceived;
        static bool bDiagLastResponsePositive;

        // States of State Machine
        static State* DIAGController::State_CAN_Idle;
        static State* DIAGController::State_CAN_Prepare;
        static State* DIAGController::State_CAN_SendingReq;
        static State* DIAGController::State_CAN_AwaitingResp; 
        // State functions
        static void CAN_Idle(); 
        static void CAN_Prepare();
        static void CAN_SendingReq();
        static void CAN_AwaitingResp();
        // Transitions of State Machine
        static bool CAN_IdleToPrepare();
        static bool CAN_PrepareToSending();
        static bool CAN_SendingToAwaitingResp();
        static bool CAN_AwaitingToSending(); 
        static bool CAN_Finalizing();
        static bool CAN_Failure();

        // Logic functions to send Diag Request and get Diag Response
        static int SendDiagRequest(uint8_t data[], int data_size);
        static void diagResponseReceived (const CANFDMessage & inMessage);
};




#endif