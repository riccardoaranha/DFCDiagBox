#include <ACAN2517FD.h>
#include <SPI.h>
#include <cppQueue.h>

// Board definition
static const byte MCP2517_CS  = 17 ; // CS input of MCP2517
static const byte MCP2517_INT =  7 ; // INT output of MCP2517
static const ACAN2517FDSettings::Oscillator inOscillator = ACAN2517FDSettings::OSC_20MHz; //Board Oscillator

// CAN communication parameters
const uint32_t inArbitrationBitRate = 500UL * 1000UL; // 500 Kbps
const DataBitRateFactor inDataBitRateFactor = DataBitRateFactor::x4; // 500 Kbps * 4 = 2Mbps
const uint32_t FrameID_Request  = 0x7C3;
const uint32_t FrameID_Response = 0x7C9;


// ACAN2517FD Driver object
ACAN2517FD can (MCP2517_CS, SPI, MCP2517_INT) ;




//Logic implementation

const uint8_t reqshort[2] = { 0x10, 0x03};
const uint8_t reqlong[40] = { 0x2E, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                              0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                              0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                              0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

cppQueue q(sizeof(uint8_t *), 2, FIFO);

// Send a Diag Request, treating the 
int SendDiagRequest(const uint8_t data[], const int data_size) {
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
    return ok;
}

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
    // Begin SPI
    SPI.begin () ;
    //--- Configure ACAN2517FD
    Serial.print ("sizeof (ACAN2517FDSettings): ") ;
    Serial.print (sizeof (ACAN2517FDSettings)) ;
    Serial.println (" bytes") ;
    Serial.println ("Configure ACAN2517FD") ;
    ACAN2517FDSettings settings (inOscillator, inArbitrationBitRate, inDataBitRateFactor) ;
    settings.mRequestedMode = ACAN2517FDSettings::NormalFD;
    settings.mDriverTransmitFIFOSize = 1 ;
    settings.mDriverReceiveFIFOSize = 1 ;
    
    //--- RAM Usage
    Serial.print ("MCP2517FD RAM Usage: ") ;
    Serial.print (settings.ramUsage ()) ;
    Serial.println (" bytes") ;
    //--- Begin
    const uint32_t errorCode = can.begin (settings, [] { can.isr () ; }) ;
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
    if (i = setup_Logger() != 0) { return i; }
    if (i = setup_CAN() != 0) { return i; }

    return 0;
}

//——————————————————————————————————————————————————————————————————————————————
//   LOOP
//——————————————————————————————————————————————————————————————————————————————

static uint32_t gSendDate = 0 ;
static uint32_t gReceiveDate = 0 ;
static uint32_t gReceivedFrameCount = 0 ;
static uint32_t gSentFrameCount = 0 ;

void loop_CAN() {
CANFDMessage frame ;

if (gSendDate < millis ()) {
    gSendDate += 2000 ;
    const bool ok = gSentFrameCount % 2 == 0 ? SendDiagRequest(reqshort, sizeof(reqshort)) : SendDiagRequest(reqlong, sizeof(reqlong));
    if (ok) {
    gSentFrameCount += 1 ;
    Serial.print ("Sent: ") ;
    Serial.print (gSentFrameCount) ;
    }else{
    Serial.print ("Send failure") ;
    }


    Serial.print (", receive overflows: ") ;
    Serial.println (can.hardwareReceiveBufferOverflowCount ()) ;
}
if (gReceiveDate < millis ()) {
    gReceiveDate += 4567 ;
    while (can.available ()) {
    can.receive (frame) ;
    gReceivedFrameCount ++ ;
    Serial.print ("Received: ") ;
    Serial.println (gReceivedFrameCount) ;
    }
}
}
//——————————————————————————————————————————————————————————————————————————————

void loop () {
    loop_CAN();
}
