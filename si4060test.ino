#include <SPI.h>
#include "si4x6x.h"

enum Mode {
  MODE_IDLE = 0,
  MODE_RX = 1,
  MODE_TX = 2
};

Mode mode;

const int pinCS = 10;
const int pinBuzzer = 7;
const int pinLED = 8;

const uint32_t xoFrequency = 26000000UL;


Si4x6x tx(pinCS, xoFrequency, false);


uint8_t enqueueRTTY7Bit(const char *str) {
  uint8_t ex = 0;
  uint8_t nBits = 0;
  uint8_t count = 0;
  
  ex = 0xFF;
  tx.writeTX(&ex, 1);
  count++;
  
  while (*str) {
    uint16_t c = (*str) & 0x7F;   // Take lower 7 bits   
    //c = (c << 2) | 0b11;  // Add 2 stop bits
    c = (c << 1) | 0x300;
    
    for (uint8_t idx = 0; idx < 10; idx++) {
      uint8_t bit = (c & 1) ? 1 : 0;
      c >>= 1;
      ex <<= 1;
      ex |= bit;
      nBits++;
      if (nBits == 8) {
        nBits = 0;
        tx.writeTX(&ex, 1);
        Serial.print(ex, HEX);
        Serial.print(' ');
        count++;
        ex = 0;
      }
    }
    
    str++;
  }
  
  if (nBits != 0) {
    while (nBits < 8) {
      ex <<= 1;
      ex |= 1;
      nBits++;
    }
    tx.writeTX(&ex, 1);
    Serial.print(ex, HEX);
    Serial.print(' ');
    count++;
  }
  
  ex = 0xFF;
  tx.writeTX(&ex, 1);
  count++;

  Serial.println();

  return count;
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

  for (uint8_t nTry = 3; nTry > 0; nTry--) {
    tx.powerUp();
    delay(100);
  }

  tx.configureGPIO(0, 0, 0, 0, 0, /* sdo */ 0x40 | 0x0B, 0);
  //tx.configureGPIO(0, 0, 0, 0, 0, /* sdo */ 0, 0);

  Si4x6x::PartInfo info;
  tx.getPartInfo(info);
  Serial.print("Part ID: "); Serial.println(info.getPartID(), HEX);
  switch (info.getPartID()) {
    case 0x4362: Serial.println("Silabs Si4362 detected, switching to RX mode"); mode = MODE_RX; break;
    case 0x4060: Serial.println("Silabs Si4060 detected, switching to TX mode"); mode = MODE_TX; break;
    default:
      mode = MODE_IDLE; 
      break;
  }


  tx.clearIRQ();
  tx.disableRadio();
  delay(500);

  
  if (mode == MODE_TX) tx.setPowerLevel(0x10);

  tx.setNCOModulo(Si4x6xBase::kModulo10);
  tx.setDataRate(50);
  tx.setDeviation(1300);
  
  tx.setModulation(Si4x6xBase::kMod2FSK, Si4x6xBase::kSourceFIFO);
  //tx.setModulation(Si4x6xBase::kMod2GFSK, Si4x6xBase::kSourcePN9);

  //tx.setPreambleLength(0);  // Disable packet preamble
  //tx.setSync(0x80);         // Disable sync word

  delay(100);

  tx.setFrequency(434.000 * 1E6);

  delay(500); 

  if (mode == MODE_RX) {
    tx.startRX(0, 5);
  }
}

void loop() {
  //uint8_t len = enqueueRTTY7Bit("abcdefghijklmnopqrstuvwxyz 1234567890\r\n");

  if (mode == MODE_TX) {
    int16_t temp = tx.getTemperature();
    Serial.print("Temperature: "); Serial.print(temp / 10); Serial.print('.'); Serial.println(temp % 10);
    
    Serial.println("Transmitting...");
  
    //tx.flushTX();
  
    uint8_t data[] = { 0x03, 0xAA, 0x2F, 0x2F, 0xBB };
  
    tx.clearIRQ();
    
    tx.writeTX(data, sizeof(data)); 
    //tx.startTX(0, sizeof(data));
    tx.startTX(0, sizeof(data));
    
    //tx.enableRadio();


    digitalWrite(pinLED, HIGH);
    delay(100);
    digitalWrite(pinLED, LOW);
  
    delay(3000);
  }
  else {
    Si4x6x::ModemStatus modemStatus;
    tx.getModemStatus(modemStatus);
    //tx.getChipStatus();

    Serial.print("RSSI=");
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
    

    Si4x6x::IRQStatus irqStatus;
    tx.clearIRQ(irqStatus);


    /*
    Serial.print("IRQ: ");
    for (uint8_t idx = 0; idx < 8; idx++) {
      Serial.print(irqStatus.rawData[idx], HEX);
      Serial.print(' ');
    }
    Serial.println();
    */

    if (irqStatus.isPacketRXPending())
    {
      uint8_t rxCount = tx.getAvailableRX();
      Serial.print("Available: ");
      Serial.println(rxCount);

      Serial.print("Packet: ");
      
      uint8_t packet[5];
      tx.readRX(packet, 5);

      for (uint8_t idx = 0; idx < 5; idx++) {
        Serial.print(packet[idx], HEX);
        Serial.print(' ');
      }
      Serial.println();
      
      //digitalWrite(pinBuzzer, HIGH);
      //delay(250);
      //digitalWrite(pinBuzzer, LOW);
    }
    if (irqStatus.isCRCErrorPending()) {
      tx.flushRX();
    }

    delay(500);
  }
}
