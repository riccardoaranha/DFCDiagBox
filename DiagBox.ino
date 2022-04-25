#include <ACAN2517FD.h>
#include <SPI.h>
#include <ArduinoQueue.h>
#include <StateMachine.h>


// Board definition
static const byte MCP2517_CS  = 17 ; // CS input of MCP2517
static const byte MCP2517_INT =  7 ; // INT output of MCP2517
static const ACAN2517FDSettings::Oscillator inOscillator = ACAN2517FDSettings::OSC_20MHz; //Board Oscillator

// CAN communication parameters
const uint32_t inArbitrationBitRate = 500UL * 1000UL; // 500 Kbps
const DataBitRateFactor inDataBitRateFactor = DataBitRateFactor::x4; // 500 Kbps * 4 = 2Mbps
const uint32_t FrameID_Request  = 0x7C3;
const uint32_t FrameID_Response = 0x7C9;
const uint32_t TesterPresent_Timer = 10000;
const uint32_t DiagTimeout = 3000;

uint8_t TesterPresent[2] = { 0x3E, 0x00 };
uint8_t reqshort[2] = { 0x10, 0x03};
uint8_t reqlong[40] = { 0x2E, 0x10, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
                        0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10, 0x11,
                        0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1A, 0x1B,
                        0x1C, 0x1D, 0x1E, 0x1F, 0x20, 0x21, 0x22, 0x23, 0x24, 0x25};

// ACAN2517FD Driver object
ACAN2517FD can (MCP2517_CS, SPI, MCP2517_INT) ;


//Logic implementation
static uint8_t* lastDiagRequest;
static uint32_t inDiagRequestSentTime;
static bool bDiagResponseReceived;
static bool bDiagLastResponsePositive;


// Send a Diag Request, treating the 
int SendDiagRequest(uint8_t data[], int data_size) {
    CANFDMessage frame ;
    bool ok = true ;
    frame.id = FrameID_Request ;
    frame.len = 8;
    if (data_size <= 7) {
        frame.data[0] = data_size;
        for (int i = 0; i < data_size; i++) {
            frame.data[i + 1] = data[i];
        }
        Serial.print  ("Sending short frame ");
        Serial.print  (frame.id, HEX); 
        Serial.print  (" - "); 
        Serial.print  (frame.data[0], HEX);  Serial.print (" "); 
        Serial.print  (frame.data[1], HEX);  Serial.print (" "); 
        Serial.print  (frame.data[2], HEX);  Serial.print (" "); 
        Serial.print  (frame.data[3], HEX);  Serial.print (" "); 
        Serial.print  (frame.data[4], HEX);  Serial.print (" "); 
        Serial.print  (frame.data[5], HEX);  Serial.print (" "); 
        Serial.print  (frame.data[6], HEX);  Serial.print (" "); 
        Serial.println(frame.data[7], HEX); 
        ok = can.tryToSend (frame) ;
    } else {
        uint8_t byte0 = 0x10;
        int i, j;
        j = 0;
        frame.data[0] = byte0;
        frame.data[1] = data_size;
        for (i = 0; i < 6; i++) {
            frame.data[i + 2] = data[i];
        }
        Serial.print  ("Sending long frame ");
        Serial.print  (frame.id, HEX); 
        Serial.print  (" - "); 
        Serial.print  (frame.data[0], HEX);  Serial.print (" "); 
        Serial.print  (frame.data[1], HEX);  Serial.print (" "); 
        Serial.print  (frame.data[2], HEX);  Serial.print (" "); 
        Serial.print  (frame.data[3], HEX);  Serial.print (" "); 
        Serial.print  (frame.data[4], HEX);  Serial.print (" "); 
        Serial.print  (frame.data[5], HEX);  Serial.print (" "); 
        Serial.print  (frame.data[6], HEX);  Serial.print (" "); 
        Serial.println(frame.data[7], HEX); 
        ok = ok && can.tryToSend (frame) ;

        byte0 = 0x21 ;
        while (i < data_size) {
            frame.id = FrameID_Request ;
            frame.len = 8;
            frame.data[0] = byte0;
            for (j = 1; j < 8 && i < data_size; j++) {
                frame.data[j] = data[i];
                i++;
            }
            Serial.print  ("Sending long frame ");
            Serial.print  (frame.id, HEX); 
            Serial.print  (" - "); 
            Serial.print  (frame.data[0], HEX);  Serial.print (" "); 
            Serial.print  (frame.data[1], HEX);  Serial.print (" "); 
            Serial.print  (frame.data[2], HEX);  Serial.print (" "); 
            Serial.print  (frame.data[3], HEX);  Serial.print (" "); 
            Serial.print  (frame.data[4], HEX);  Serial.print (" "); 
            Serial.print  (frame.data[5], HEX);  Serial.print (" "); 
            Serial.print  (frame.data[6], HEX);  Serial.print (" "); 
            Serial.println(frame.data[7], HEX); 
            ok = ok && can.tryToSend (frame) ;
            byte0++ ;
            
        }
    }
    if (data != TesterPresent) {
        if (ok) {
            lastDiagRequest = data;
            inDiagRequestSentTime = millis();
        }
        bDiagResponseReceived = false;
        bDiagLastResponsePositive = false;
    }
    return ok;
}

void diagResponseReceived (const CANFDMessage & inMessage) {
    Serial.print  ("Received Diag Response ");
    Serial.print  (inMessage.id, HEX); 
    Serial.print  (" - "); 
    Serial.print  (inMessage.data[0], HEX);  Serial.print (" "); 
    Serial.print  (inMessage.data[1], HEX);  Serial.print (" "); 
    Serial.print  (inMessage.data[2], HEX);  Serial.print (" "); 
    Serial.print  (inMessage.data[3], HEX);  Serial.print (" "); 
    Serial.print  (inMessage.data[4], HEX);  Serial.print (" "); 
    Serial.print  (inMessage.data[5], HEX);  Serial.print (" "); 
    Serial.print  (inMessage.data[6], HEX);  Serial.print (" "); 
    Serial.print(inMessage.data[7], HEX); 
    if (inMessage.data[1] == lastDiagRequest[0] + 0x40) {
        Serial.println(". Positive response."); 
        bDiagResponseReceived = true;
        bDiagLastResponsePositive = true;
    } else if (inMessage.data[1] == 0x7F && inMessage.data[2] == lastDiagRequest[0]) {
        Serial.println(". Negative response."); 
        bDiagResponseReceived = true;
        bDiagLastResponsePositive = false;
    }
    else {
        Serial.println(". Not able to evaluate response.");
    }
}

typedef struct request {
    long size;
    uint8_t *data;
} CANRequest;

static ArduinoQueue<CANRequest> diagQueue(32);
static bool CAN_OverallSuccess;
StateMachine CAN_machine = StateMachine();

void CAN_Idle(){
    Serial.println("CAN - Idle");
    while (!diagQueue.isEmpty()) { 
        diagQueue.dequeue(); 
    }
}

void CAN_Prepare() {
    Serial.println("CAN - Preparing");
    if (CAN_machine.executeOnce) {
        CAN_OverallSuccess = true;
        CANRequest req;
        req.size = sizeof(reqshort);
        req.data = reqshort;
        diagQueue.enqueue(req);
        CANRequest req2;
        req2.size = sizeof(reqlong);
        req2.data = reqlong;
        diagQueue.enqueue(req2);
    }
}

void CAN_SendingReq() {
    if (CAN_machine.executeOnce) {
        CANRequest req = diagQueue.dequeue();
        CAN_OverallSuccess = CAN_OverallSuccess && SendDiagRequest(req.data, req.size);
    }
}

void CAN_AwaitingResp() {
    if (millis() > inDiagRequestSentTime + DiagTimeout) {
        Serial.println("CAN - Timeout awaiting response");
        CAN_OverallSuccess = false;
    }
}

bool CAN_IdleToPrepare() {
    //Serial.println("Idle to prepare.");
    //TODO: Change this delay to some input 
    return true;
}

bool CAN_PrepareToSending() {
    //Serial.println("Prepare to Sending");    
    return true;
}

bool CAN_SendingToAwaitingResp() {
    return CAN_OverallSuccess;
}

bool CAN_AwaitingToSending() {
    if (CAN_OverallSuccess && bDiagResponseReceived && bDiagLastResponsePositive && !diagQueue.isEmpty()) { 
        return true; 
    } else { 
        return false; 
    }
}

bool CAN_Finalizing() {
    if (CAN_OverallSuccess && bDiagResponseReceived && bDiagLastResponsePositive && diagQueue.isEmpty()) {
        Serial.println("Finished with success. Returning to Idle.");    
        return true;
    } else {
        return false; 
    }
}

bool CAN_Failure() {
    if (!CAN_OverallSuccess) {
        Serial.println("CAN Failure. Returning to Idle");
        return true;
    }
    return false;
}

State* State_CAN_Idle = CAN_machine.addState(&CAN_Idle); 
State* State_CAN_Prepare = CAN_machine.addState(&CAN_Prepare); 
State* State_CAN_SendingReq = CAN_machine.addState(&CAN_SendingReq); 
State* State_CAN_AwaitingResp = CAN_machine.addState(&CAN_AwaitingResp); 


//——————————————————————————————————————————————————————————————————————————————
//   SETUP
//——————————————————————————————————————————————————————————————————————————————
int setup_Logger () {
    Serial.begin (115200) ;
    while (!Serial) {
        delay (50) ;
    }
    return 0;
}



int setup_CAN () {
    // State Machine Transitions Setup
    State_CAN_Idle->addTransition(&CAN_IdleToPrepare, State_CAN_Prepare);    
    State_CAN_Prepare->addTransition(&CAN_PrepareToSending, State_CAN_SendingReq); 
    State_CAN_SendingReq->addTransition(&CAN_SendingToAwaitingResp, State_CAN_AwaitingResp); 
    State_CAN_AwaitingResp->addTransition(&CAN_AwaitingToSending, State_CAN_SendingReq);
    State_CAN_AwaitingResp->addTransition(&CAN_Finalizing, State_CAN_Idle);  
    State_CAN_SendingReq->addTransition(&CAN_Failure, State_CAN_Idle);  
    State_CAN_AwaitingResp->addTransition(&CAN_Failure, State_CAN_Idle);
    
    // Begin SPI
    SPI.begin () ;
    //--- Configure ACAN2517FD
    Serial.print ("sizeof (ACAN2517FDSettings): ") ;
    Serial.print (sizeof (ACAN2517FDSettings), DEC) ;
    Serial.println (" bytes") ;
    Serial.println ("Configure ACAN2517FD") ;
    ACAN2517FDSettings settings (inOscillator, inArbitrationBitRate, inDataBitRateFactor) ;
    settings.mRequestedMode = ACAN2517FDSettings::NormalFD;
    settings.mDriverTransmitFIFOSize = 1 ;
    settings.mDriverReceiveFIFOSize = 1 ;

    ACAN2517FDFilters filters ;
    // Filter to treat only the DiagResponse message 
    filters.appendFrameFilter (kStandard, FrameID_Response, diagResponseReceived) ;
    
    //--- RAM Usage
    Serial.print ("MCP2517FD RAM Usage: ") ;
    Serial.print (settings.ramUsage ()) ;
    Serial.println (" bytes") ;
    //--- Begin
    const uint32_t errorCode = can.begin (settings, [] { can.isr () ; }, filters) ;
    if (errorCode == 0) {
        Serial.print ("Bit Rate prescaler: ") ;
        Serial.println (settings.mBitRatePrescaler) ;
        Serial.print ("Arbitration Phase segment 1: ") ;
        Serial.println (settings.mArbitrationPhaseSegment1) ;
        Serial.print ("Arbitration Phase segment 2: ") ;
        Serial.println (settings.mArbitrationPhaseSegment2) ;
        Serial.print ("Arbitration SJW:") ;
        Serial.println (settings.mArbitrationSJW) ;
        Serial.print ("Actual Arbitration Bit Rate: ") ;
        Serial.print (settings.actualArbitrationBitRate ()) ;
        Serial.println (" bit/s") ;
        Serial.print ("Exact Arbitration Bit Rate ? ") ;
        Serial.println (settings.exactArbitrationBitRate () ? "yes" : "no") ;
        Serial.print ("Arbitration Sample point: ") ;
        Serial.print (settings.arbitrationSamplePointFromBitStart ()) ;
        Serial.println ("%") ;
        return 0;
    } else {
        Serial.print ("Configuration error 0x") ;
        Serial.println (errorCode, HEX) ;
        return -1;
    }
}

void setup () {
    int i; 
    if (i = setup_Logger() != 0) { return; }
    if (i = setup_CAN() != 0) { return; }

}

//——————————————————————————————————————————————————————————————————————————————
//   LOOP
//——————————————————————————————————————————————————————————————————————————————



static uint32_t gSendTesterPresentDate = 0 ;

void loop_CAN() {
    // Send Tester Present
    if (TesterPresent_Timer > 0 && gSendTesterPresentDate < millis ()) {
        SendDiagRequest(TesterPresent, sizeof(TesterPresent));
        gSendTesterPresentDate += TesterPresent_Timer;
    }
    can.dispatchReceivedMessage ();
    CAN_machine.run();
}
//——————————————————————————————————————————————————————————————————————————————

void loop () {
    loop_CAN();
    delay(10);
}
