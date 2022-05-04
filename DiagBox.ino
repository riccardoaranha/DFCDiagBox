#include "DIAGController.h"

// CAN communication parameters
const uint32_t arbitrationBitRate = 500UL * 1000UL; // 500 Kbps
const DataBitRateFactor dataBitRateFactor = DataBitRateFactor::x4; // 500 Kbps * 4 = 2Mbps
const uint32_t frameID_Request  = 0x7C3;
const uint32_t frameID_Response = 0x7C9;
const uint32_t testerPresent_Timer = 10000;
const uint32_t diagTimeout = 3000;

// Messages
uint8_t ExtendedSession[2] = { 0x10, 0x03};
uint8_t Read1001[3] = { 0x22, 0x10, 0x01};
uint8_t LongReq[40] = { 0x2E, 0x10, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
                        0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10, 0x11,
                        0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1A, 0x1B,
                        0x1C, 0x1D, 0x1E, 0x1F, 0x20, 0x21, 0x22, 0x23, 0x24, 0x25};


void SendRequest1() {
    DIAGController::addMessage(ExtendedSession, sizeof(ExtendedSession));
    DIAGController::addMessage(Read1001, sizeof(Read1001));
    DIAGController::startRoutine();

}

void SendRequest2() {
    DIAGController::addMessage(ExtendedSession, sizeof(ExtendedSession));
    DIAGController::addMessage(LongReq, sizeof(LongReq));
    DIAGController::startRoutine();
}

//——————————————————————————————————————————————————————————————————————————————
//   SETUP
//——————————————————————————————————————————————————————————————————————————————
void setup () {
    Serial.begin (115200) ;
    while (!Serial) {
        delay (50) ;
    }
    
    // Setup the CAN interface
    DIAGController::setup(arbitrationBitRate, 
                        dataBitRateFactor, 
                        frameID_Request, 
                        frameID_Response, 
                        testerPresent_Timer, 
                         diagTimeout);
}

//——————————————————————————————————————————————————————————————————————————————
//   LOOP
//——————————————————————————————————————————————————————————————————————————————

void loop () {
    // Loop the CAN interface
    DIAGController::loop();
    delay(50);
}
