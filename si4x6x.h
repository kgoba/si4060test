#include "Arduino.h"
#include <SPI.h>

class SPIDevice {
public:
  SPIDevice(int pinCS) : _pinCS(pinCS) {}

  void select() {
    digitalWrite(_pinCS, LOW);
    SPI.beginTransaction(SPISettings(14000000, MSBFIRST, SPI_MODE0));
  }

  uint8_t write(uint8_t x) {
    return SPI.transfer(x);
  }

  void release() {
    SPI.endTransaction();
    digitalWrite(_pinCS, HIGH);
  }

private:
  int _pinCS;
};

class Si4x6xBase {
public:
  enum ModulationType {
    kModCW    = 0,
    kModOOK   = 1,
    kMod2FSK  = 2,
    kMod2GFSK = 3,
    kMod4FSK  = 4,
    kMod4GFSK = 5
  };
  
  enum ModulationSource {
    kSourceFIFO    = 0,
    kSourceDirect  = 1,
    kSourcePN9     = 2
  };  

  enum NCOModulo {
    kModulo10  = 0,
    kModulo20  = 2,
    kModulo40  = 1
  };  

  enum State {
    kStateSleep     = 1,
    kStateSPIActive = 2,
    kStateReady     = 3,
    kStateTXTune    = 5,
    kStateRXTune    = 6,
    kStateTX        = 7,
    kStateRX        = 8
  };
};



class Si4x6x : public Si4x6xBase, SPIDevice {
public:
  struct PartInfo {
    uint16_t getPartID() {
      return ((rawData[1] << 8) | rawData[2]);
    }
  
    uint8_t getRevision() {
      return rawData[0];
    }
    
    uint8_t   rawData[8];
  };

  struct IRQStatus {
    bool isPacketRXPending() {
      return rawData[2] & (1 << 4);
    }

    bool isPacketRX() {
      return rawData[3] & (1 << 4);
    }

    bool isCRCErrorPending() {
      return rawData[2] & (1 << 3);
    }
    
    uint8_t   rawData[8];
  };  

  struct ModemStatus {
    
    uint8_t getCurrentRSSI() {
      return rawData[2];
    }

    uint8_t getLatchedRSSI() {
      return rawData[3];
    }

    bool isSyncDetect() {
      return rawData[1] & (1 << 0);
    }
    
    bool isPreambleDetect() {
      return rawData[1] & (1 << 1);
    }

    bool isInvalidSync() {
      return rawData[1] & (1 << 5);
    }

    bool isInvalidPreamble() {
      return rawData[1] & (1 << 2);
    }
        
    bool isRSSI() {
      return rawData[1] & (1 << 3);
    }

    bool isSyncDetectPending() {
      return rawData[0] & (1 << 0);
    }
    
    bool isPreambleDetectPending() {
      return rawData[0] & (1 << 1);
    }

    bool isInvalidSyncPending() {
      return rawData[0] & (1 << 5);
    }

    bool isInvalidPreamblePending() {
      return rawData[0] & (1 << 2);
    }
        
    bool isRSSIPending() {
      return rawData[0] & (1 << 3);
    }
    
    uint8_t   rawData[8];
  };     

  struct ChipStatus {
        
    uint8_t   rawData[3];
  };  
  
  //Si4x6x(SPI &spi, PinName pinCS, uint32_t xtalFrequency, bool isTCXO = false);
  Si4x6x(int pinCS, uint32_t xtalFrequency, bool isTCXO = false);

  void getPartInfo(PartInfo &info);
  
  void shutdown();
  void powerUp(uint8_t bootOptions = 0x01);
  void setPowerLevel(uint8_t level);
  void configureGPIO(uint8_t gpio0, uint8_t gpio1, uint8_t gpio2, uint8_t gpio3, uint8_t nirq, uint8_t sdo, uint8_t genConfig);

  void enableTX();
  void disableRadio();

  void setFrequency(uint32_t freq);
  void startTX(uint8_t channel, uint16_t pktLength = 0, State txCompleteState = 0);
  void startRX(uint8_t channel, uint16_t pktLength = 0);

  void setModulation(ModulationType modType, ModulationSource modSource = kSourceFIFO, uint8_t txDirectModeGPIO = 0, uint8_t txDirectModeType = 0);
  void setDataRate(uint32_t dataRate);
  void setDeviation(uint32_t deviation);
  
  void setNCOModulo(NCOModulo osr);
  
  void setPreambleLength(uint8_t length);
  void setSync(uint8_t config);

  void flushTX();
  void writeTX(const uint8_t *data, uint8_t length);

  void flushRX();
  void readRX(uint8_t *data, uint8_t length);

  uint8_t getAvailableRX();

  void clearIRQ();
  void clearIRQ(IRQStatus &status);
  void getPHStatus();
  void getModemStatus(ModemStatus &status);  
  void getChipStatus(ChipStatus &status);
  
  int16_t getTemperature(); 
  
private: 
  bool waitForCTS(uint16_t timeout = 10);
  bool waitForResponse(uint8_t *buf = 0, uint8_t len = 0, uint16_t timeout = 10);
  bool sendCommand(uint8_t cmd, const uint8_t *params = 0, uint8_t paramsLength = 0, uint8_t *reply = 0, uint8_t replyLength = 0);

  void setState(State state);

  //SPI         _spi;
  //DigitalOut  _cs;
  
  uint32_t    _xtalFrequency;
  bool        _isTCXO;
  uint8_t     _outDiv;
  uint32_t    _pfdFrequency;

  /*
  uint8_t     _intPend,   _intStatus;
  uint8_t     _phPend,    _phStatus;
  uint8_t     _modemPend, _modemStatus;
  uint8_t     _chipPend,  _chipStatus;
  */
};

