#include "DIAGController.h"

// ACAN2517FD Driver object
static ACAN2517FD can(MCP2517_CS, SPI, MCP2517_INT);
// Queue of messages to implement
static ArduinoQueue<DIAGRequest> diagQueue(QUEUE_MAX_SIZE);
// State Machine
static StateMachine machine; 

//——————————————————————————————————————————————————————————————————————————————
//   DIAG Logic implementation
//——————————————————————————————————————————————————————————————————————————————
int DIAGController::SendDiagRequest(uint8_t data[], int data_size) {
    CANFDMessage frame ;
    bool ok = true ;
    frame.id = mFrameID_Request ;
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
        if (data != TesterPresent) {
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
        }
        ok = ok && can.tryToSend (frame) ;

        byte0 = 0x21 ;
        while (i < data_size) {
            frame.id = mFrameID_Request ;
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

void DIAGController::diagResponseReceived (const CANFDMessage & inMessage) {
    // Ignoring Tester present positive response
    if (inMessage.data[0] == TesterPresent[0] + 0x40) {
        return;
    }
    // Print Message
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
        // Positive response for the last request
        Serial.println(". Positive response."); 
        bDiagResponseReceived = true;
        bDiagLastResponsePositive = true;
    } else if (inMessage.data[1] == 0x7F && inMessage.data[2] == lastDiagRequest[0]) {
        // Negative response for the last request
        Serial.println(". Negative response."); 
        bDiagResponseReceived = true;
        bDiagLastResponsePositive = false;
    }
    else {
        // Other response. Could be partial ack, g.e.
        Serial.println(". Not able to evaluate response.");
    }
}

//——————————————————————————————————————————————————————————————————————————————
//   State Machine
//——————————————————————————————————————————————————————————————————————————————
void DIAGController::CAN_Idle(){
    Serial.println("CAN - Idle");
    // Empty the queue;
    while (!diagQueue.isEmpty()) { 
        diagQueue.dequeue(); 
    }
}

void DIAGController::CAN_Prepare() {
    Serial.println("CAN - Preparing");
    if (machine.executeOnce) {
        OverallSuccess = true;

        // TODO: redo this part
        DIAGRequest req;
        //req.size = sizeof(reqshort);
        //req.data = reqshort;
        diagQueue.enqueue(req);
        DIAGRequest req2;
        //req2.size = sizeof(reqlong);
        //req2.data = reqlong;
        diagQueue.enqueue(req2);
    }
}

void DIAGController::CAN_SendingReq() {
    if (machine.executeOnce) {
        DIAGRequest req = diagQueue.dequeue();
        OverallSuccess = OverallSuccess && SendDiagRequest(req.data, req.size);
    }
}

void DIAGController::CAN_AwaitingResp() {
    if (millis() > inDiagRequestSentTime + mDiagTimeout) {
        Serial.println("CAN - Timeout awaiting response");
        OverallSuccess = false;
    }
}

bool DIAGController::CAN_IdleToPrepare() {
    //Serial.println("Idle to prepare.");
    //TODO: Change this to some input
    return true;
}

bool DIAGController::CAN_PrepareToSending() {
    // Is ready if queue to send is done;
    return !diagQueue.isEmpty();
}

bool DIAGController::CAN_SendingToAwaitingResp() {
    return OverallSuccess;
}

bool DIAGController::CAN_AwaitingToSending() {
    if (OverallSuccess && bDiagResponseReceived && bDiagLastResponsePositive && !diagQueue.isEmpty()) { 
        return true; 
    } else { 
        return false; 
    }
}

bool DIAGController::CAN_Finalizing() {
    if (OverallSuccess && bDiagResponseReceived && bDiagLastResponsePositive && diagQueue.isEmpty()) {
        Serial.println("Finished with success. Returning to Idle.");    
        return true;
    } else {
        return false; 
    }
}

bool DIAGController::CAN_Failure() {
    if (!OverallSuccess) {
        Serial.println("CAN Failure. Returning to Idle");
        return true;
    }
    return false;
}

//——————————————————————————————————————————————————————————————————————————————
//   Setup and Loop
//——————————————————————————————————————————————————————————————————————————————
int DIAGController::setup(uint32_t inArbitrationBitRate,
                         DataBitRateFactor inDataBitRateFactor,
                         uint32_t inFrameID_Request, 
                         uint32_t inFrameID_Response, 
                         uint32_t inTesterPresent_Timer = 3000, 
                         uint32_t inDiagTimeout = 5000) {
    mArbitrationBitRate = inArbitrationBitRate;
    mDataBitRateFactor = inDataBitRateFactor;
    mFrameID_Request = inFrameID_Request;
    mFrameID_Response = inFrameID_Response;
    mTesterPresent_Timer = inTesterPresent_Timer;
    mDiagTimeout = inDiagTimeout;
    
    //Queue initialization
    //diagQueue = ArduinoQueue<CANRequest>(QUEUE_MAX_SIZE);
    
    machine = StateMachine();
    //Machine states initialization
    State_CAN_Idle = machine.addState(&CAN_Idle); 
    State_CAN_Prepare = machine.addState(&CAN_Prepare); 
    State_CAN_SendingReq = machine.addState(&CAN_SendingReq); 
    State_CAN_AwaitingResp = machine.addState(&CAN_AwaitingResp); 
    
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
    Serial.print ((long)(sizeof (ACAN2517FDSettings)), DEC) ;
    Serial.println (" bytes") ;
    Serial.println ("Configure ACAN2517FD") ;
    ACAN2517FDSettings settings (mOscillator, mArbitrationBitRate, mDataBitRateFactor) ;
    settings.mRequestedMode = ACAN2517FDSettings::NormalFD;
    settings.mDriverTransmitFIFOSize = 1 ;
    settings.mDriverReceiveFIFOSize = 1 ;

    ACAN2517FDFilters filters ;
    // Filter to treat only the DiagResponse message 
    filters.appendFrameFilter (kStandard, mFrameID_Response, diagResponseReceived) ;
    
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

void DIAGController::loop() {
    // Send Tester Present
    if (mTesterPresent_Timer > 0 && gSendTesterPresentDate < millis ()) {
        SendDiagRequest(TesterPresent, sizeof(TesterPresent));
        gSendTesterPresentDate += mTesterPresent_Timer;
    }
    can.dispatchReceivedMessage ();
    machine.run();
}

