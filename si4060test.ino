#include <SPI.h>
#include "si4x6x.h"

enum Mode {
  MODE_IDLE = 0,
  MODE_RX = 1,
  MODE_TX = 2
};

#include "radio_config_Si4362.h"
Mode mode = MODE_RX;

//#include "radio_config_Si4060.h"
//Mode mode = MODE_TX;

const int pinCS = 10;
const int pinBuzzer = 7;
const int pinLED = 8;

const uint8_t kPacketLength = 16;

const uint32_t xoFrequency = 26000000UL;
const uint8_t  xoTune      = 27;

Si446x tx(pinCS, xoFrequency);

void initModemAlt()
{  
  for (uint8_t nTry = 3; nTry > 0; nTry--) {
    Serial.println("Initializing radio...");
    uint8_t config[] = RADIO_CONFIGURATION_DATA_ARRAY;
    if (tx.configure(config)) break;
    Serial.println("Failed");
    delay(100);
  }    
}

void initModem()
{ 
  for (uint8_t nTry = 3; nTry > 0; nTry--) {
    if (mode == MODE_RX) {
      tx.powerUpXTAL();
    }
    if (mode == MODE_TX) {
      tx.powerUpTCXO();
    }
    delay(100);
  }

  if (mode == MODE_RX) {
    tx.configureGPIO(0x15, 0x11, 0, 0, 0, /* sdo */ 0x40 | 0x0B, 0);
  }
  if (mode == MODE_TX) {
    tx.configureGPIO(0x44, 0x0F, 0, 0, 0, /* sdo */ 0x40 | 0x0B, 0);
  }

  Si446x::PartInfo info;
  tx.getPartInfo(info);
  Serial.print("Part ID: "); Serial.println(info.getPartID(), HEX);
  switch (info.getPartID()) {
    case 0x4362: Serial.println("Silabs Si4362 detected, switching to RX mode"); mode = MODE_RX; break;
    case 0x4060: Serial.println("Silabs Si4060 detected, switching to TX mode"); mode = MODE_TX; break;
    default:
      mode = MODE_IDLE; 
      break;
  }

  tx.setGlobalConfig(0x60);

  if (mode == MODE_TX) {
    tx.setXOTune(0);
    //tx.setIntControl(false, false, true);   // Enable only Packet Handler interrupts
    //tx.setPHInterrupts(0x20);               // Enable PACKET_SENT interrupt
    tx.setPreambleLength(0x0A);
  }
  if (mode == MODE_RX) {
    tx.setXOTune(xoTune);
    //tx.setIntControl(false, false, true);   // Enable only Packet Handler interrupts
    //tx.setPHInterrupts(0x18);               // Enable PACKET_RX and CRC_ERROR interrupts
  }

  tx.setFrequency(434.000 * 1E6);

  tx.setPreambleConfig(0x31);
  tx.setSync(0x01, 0xB42B);
  tx.setPacketConfig(0x02);
  tx.setField1Config(0x04);

  tx.setModulation(Si446xBase::kMod2FSK, Si446xBase::kSourceFIFO);
  tx.setNCOModulo(Si446xBase::kModulo10, xoFrequency);
  tx.setDataRate(10 * 1000);
  tx.setDeviation(20000);
  tx.setModemParams(0x80, 0x08, 0x038000ul, 0x30, 0x20);
  tx.setBCRParams(1625, 20649, 40, 0x02, 0xC2);

  if (mode == MODE_TX) {
    tx.setPAConfig(0x18, 0x10, 0xC0, 0x3D);     // mode, level, duty, tc
  }
  if (mode == MODE_RX) {
    tx.setRSSIMode(0x02);
    tx.setRSSIThreshold(0x40);
    tx.setRSSIComp(0x40);
  }
}

void setup() {
  // put your setup code here, to run once:
  SPI.begin();
  pinMode(pinCS, OUTPUT);
  pinMode(pinBuzzer, OUTPUT);
  pinMode(pinLED, OUTPUT);

  digitalWrite(pinBuzzer, HIGH);
  delay(100);
  digitalWrite(pinBuzzer, LOW);

  sei();

  delay(100);

  Serial.begin(9600);
  Serial.println("Reset!");

  initModemAlt();


  tx.clearIRQ();
  //tx.disableRadio();
  //tx.clearIRQ();
  delay(500);

  if (mode == MODE_RX) {  
    tx.startRX(0, kPacketLength, Si446x::kStateNoChange, Si446x::kStateRX, Si446x::kStateRX);
  }  
}

void parseCommand(const String & line) {
  if (line.length() == 0) return;
  Serial.println(line);
  
  int delimIndex = line.indexOf(' ');
  bool parseError = false;
  if (delimIndex > 0) {
    String cmd = line.substring(0, delimIndex);
    String args = line.substring(delimIndex + 1);
    if (cmd == String("xo")) {
      tx.setXOTune(args.toInt());
    }
    if (cmd == String("tx")) {
      mode = MODE_TX;
      initModem();
    }
    if (cmd == String("rx")) {
      mode = MODE_RX;
      initModem();
    }
    else {
      parseError = true;
    }
  }
  else {    
    String cmd = line;
    //if (cmd == String("kp")) {
    //}
    parseError = true;
  }
  
  if (!parseError) {
    Serial.println("OK");
  }
  else {
    Serial.println("ERROR");
  }
}

void processConsole() {
  static uint8_t len;
  static char line[80];
  
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      line[len++] = '\0';
      parseCommand(line);
      len = 0;
      break;
    }
    else {
      if (len < 80 - 1) line[len++] = c;
    }
  }
}

void loop() {
  static uint16_t count;
  
  processConsole();
  
  if (mode == MODE_TX && count == 200) {
    count = 0;

    int16_t temp = tx.getTemperature();
    Serial.print("Temperature: "); Serial.print(temp / 10); Serial.print('.'); Serial.println(temp % 10);

    digitalWrite(pinLED, HIGH);
    delay(100);
    digitalWrite(pinLED, LOW);
    Serial.println("Transmitting...");
  
    uint8_t data[kPacketLength] = { 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f };
  

    for (uint8_t idx = 0; idx < 8; idx++) 
    {
      uint16_t nTry = 100;
      while (nTry > 0) {
        Si446x::IRQStatus irqStatus;
        tx.clearIRQ(irqStatus);

        if (irqStatus.isPacketSentPending()) {     
          break;
        }
        delay(10);
        nTry--;
      }
      tx.clearIRQ();
      tx.writeTX(data, kPacketLength); 
      tx.startTX(0, kPacketLength);      
    }
  }
  else if (mode == MODE_RX && count == 50) {
    count = 0;

    /*
    uint8_t state = tx.getState();
    Serial.print("State=");
    Serial.print(state);

    Si446x::ModemStatus modemStatus;
    tx.getModemStatus(modemStatus);
    //tx.getChipStatus();

    Serial.print(" RSSI=");
    Serial.println(modemStatus.getCurrentRSSI());


    Serial.print("Modem pending status: ");
    if (modemStatus.isRSSIPending()) Serial.print("RSSI_OK ");
    if (modemStatus.isPreambleDetectPending()) Serial.print("PRE_DET ");
    if (modemStatus.isInvalidPreamblePending()) Serial.print("PRE_INVALID ");
    if (modemStatus.isSyncDetectPending()) Serial.print("SYN_DET ");
    if (modemStatus.isInvalidSyncPending()) Serial.print("SYN_INVALID ");
    Serial.println();
    
    Serial.print("Modem status: ");
    if (modemStatus.isRSSI()) Serial.print("RSSI_OK ");
    if (modemStatus.isPreambleDetect()) Serial.print("PRE_DET ");
    if (modemStatus.isInvalidPreamble()) Serial.print("PRE_INVALID ");
    if (modemStatus.isSyncDetect()) Serial.print("SYN_DET ");
    if (modemStatus.isInvalidSync()) Serial.print("SYN_INVALID ");
    Serial.println();   
     */




    Si446x::IRQStatus irqStatus;
    tx.clearIRQ(irqStatus);

    Serial.print("IRQ: ");
    for (uint8_t idx = 0; idx < 8; idx++) {
      Serial.print(irqStatus.rawData[idx], HEX);
      Serial.print(' ');
    }
    Serial.println();

    if (irqStatus.isPacketRXPending())
    {
      uint8_t rxCount = tx.getAvailableRX();
      if (rxCount > 0) {
        Serial.print("Available: ");
        Serial.println(rxCount);
      }

      
      Serial.print("Packet: ");
      
      uint8_t packet[kPacketLength];
      tx.readRX(packet, kPacketLength);

      for (uint8_t idx = 0; idx < kPacketLength; idx++) {
        Serial.print(packet[idx], HEX);
        Serial.print(' ');
      }
      Serial.println();
      
      digitalWrite(pinBuzzer, HIGH);
      delay(250);
      digitalWrite(pinBuzzer, LOW);
    }
    if (irqStatus.isCRCErrorPending()) {
      tx.flushRX();
    }
  }

  count++;
  delay(10);
}
