#include "si4x6x.h"




/* Commands */
enum {
  SI_CMD_NOP      = 0x00,
  SI_CMD_PART_INFO    = 0x01,
  SI_CMD_POWER_UP     = 0x02,
  SI_CMD_FUNC_INFO    = 0x10,
  SI_CMD_SET_PROPERTY   = 0x11,
  SI_CMD_GET_PROPERTY   = 0x12,
  SI_CMD_GPIO_PIN_CFG   = 0x13,
  SI_CMD_GET_ADC_READING = 0x14,
  SI_CMD_FIFO_INFO    = 0x15,
  SI_CMD_PACKET_INFO    = 0x16,
  SI_CMD_IRCAL    = 0x17,
  SI_CMD_PROTOCOL_CFG    = 0x18,
  SI_CMD_GET_INT_STATUS   = 0x20,
  SI_CMD_GET_PH_STATUS   = 0x21,
  SI_CMD_GET_MODEM_STATUS   = 0x22,
  SI_CMD_GET_CHIP_STATUS   = 0x23,
  SI_CMD_START_TX   = 0x31,
  SI_CMD_START_RX   = 0x32,
  SI_CMD_REQUEST_DEVICE_STATE = 0x33,
  SI_CMD_CHANGE_STATE   = 0x34,
  SI_CMD_RX_HOP   = 0x36,
  SI_CMD_READ_CMD_BUFF    = 0x44,
  SI_CMD_FRR_A_READ   = 0x50,
  SI_CMD_FRR_B_READ   = 0x51,
  SI_CMD_FRR_C_READ   = 0x53,
  SI_CMD_FRR_D_READ   = 0x57,
  SI_CMD_WRITE_TX_FIFO   = 0x66,
  SI_CMD_READ_RX_FIFO   = 0x77,
};



Si4x6x::Si4x6x(int pinCS, uint32_t xtalFrequency, bool isTCXO) 
  : SPIDevice(pinCS), _xtalFrequency(xtalFrequency), _isTCXO(isTCXO), _outDiv(4)
{
}



/**
 * Resets the radio
 */

void Si4x6x::shutdown()
{
  // NO SDN PIN 
}


/**
 * Gets the 16 bit part number
 */
/*
static uint16_t si_trx_get_part_info(void)
{
  uint8_t buffer[3];

  buffer[0] = SI_CMD_PART_INFO;

  if (_si_trx_transfer(1, 3, buffer)) {
    return (buffer[1] << 8) | buffer[2];
  }
  else {
    return 0xFFFF;
  }
}
*/


bool Si4x6x::waitForCTS(uint16_t timeout)
{ 
  /* Poll CTS over SPI */
  while (true)
  {    
    SPIDevice::select();
    SPIDevice::write(SI_CMD_READ_CMD_BUFF);
    uint8_t r = SPIDevice::write(0xFF);
    if (r == 0xFF) break;
    SPIDevice::release();
    
    delay(1);
    if (--timeout == 0) return false;
  }

  SPIDevice::release();
  delayMicroseconds(1);
  return true;
}


bool Si4x6x::waitForResponse(uint8_t *buf, uint8_t len, uint16_t timeout)
{
  /* Poll CTS over SPI */
  while(true)
  {
    SPIDevice::select();
    SPIDevice::write(SI_CMD_READ_CMD_BUFF);
    uint8_t r = SPIDevice::write(0xFF);
    if(r == 0xFF) break;
    SPIDevice::release();
    
    delay(1);
    if (--timeout == 0) {
      //Serial.println("TIMEOUT");
      return false;
    }
  }

  //if (len > 0) Serial.print("<");
  /* Read the requested data */
  while (len > 0) {
    uint8_t x = SPIDevice::write(0xFF);
    //Serial.print(x, HEX);
    //if (len > 1) Serial.print(' ');
    //else Serial.println();
    *(buf++) = x;
    len--;
  }

  SPIDevice::release();
  delayMicroseconds(1);
  //Serial.print("SUCCESS ");
  //Serial.println(timeout);
  return true;
}


bool Si4x6x::sendCommand(uint8_t cmd, const uint8_t *data, uint8_t dataLength, uint8_t *reply, uint8_t replyLength)
{
  /* Set SS low to select chip */
  SPIDevice::select();

  //Serial.print(">");
  /* Send the command and data */
  SPIDevice::write(cmd);
  //Serial.print(cmd, HEX);
  //if (dataLength > 0) Serial.print(' ');
  for(; dataLength; dataLength--) {
    uint8_t x = *(data++);
    SPIDevice::write(x);
    //Serial.print(x, HEX);
    //if (dataLength > 1) {
    //  Serial.print(' ');
    //}
    //else {
    //  Serial.println();
    //}
  }

  /* Unselect the chip */
  delayMicroseconds(1); /* Select hold time min 50 ns */
  SPIDevice::release();

  delayMicroseconds(1);
  return waitForResponse(reply, replyLength);
}


void Si4x6x::powerUp(uint8_t bootOptions) 
{
  uint8_t xtalOptions = (_isTCXO) ? 0x01 : 0x00;

  uint8_t data[6] = {
    bootOptions, xtalOptions,
    (uint8_t)(_xtalFrequency >> 24),
    (uint8_t)(_xtalFrequency >> 16),
    (uint8_t)(_xtalFrequency >> 8),
    (uint8_t)(_xtalFrequency)
  };

  sendCommand(SI_CMD_POWER_UP, data, sizeof(data));
}


int16_t Si4x6x::getTemperature()
{
  uint8_t reply[8];

  uint8_t data[] = { 
    0x10
  };

  sendCommand(SI_CMD_GET_ADC_READING, data, sizeof(data), reply, 8);

  /* Calculate the temperature in C * 10 */
  int32_t temp;
  temp  = (reply[4] << 8) | reply[5];
  temp *= 568;
  temp /= 256;
  temp -= 2970;

  return temp;
}


void Si4x6x::setModulation(ModulationType modType, ModulationSource modSource, uint8_t txDirectModeGPIO, uint8_t txDirectModeType)
{
  uint8_t mode = 
    ((txDirectModeType & 1) << 7) |
    ((txDirectModeGPIO & 3) << 5) |
    ((modSource & 3)        << 3) |
    ((modType & 7)          << 0);
    
  uint8_t data[] = { 
    0x20, 0x02, 0x00, mode, 0x00
  };
  sendCommand(SI_CMD_SET_PROPERTY, data, 5);
}


void Si4x6x::setState(Si4x6x::State state)
{
  uint8_t data[] = { 
    (uint8_t)state
  };
  sendCommand(SI_CMD_CHANGE_STATE, data, 1);
}


void Si4x6x::enableTX(void)
{
  setState(kStateTX);
}


void Si4x6x::disableRadio(void)
{
  setState(kStateReady);
}

void Si4x6x::setFrequency(uint32_t freq)
{
  // Set the output divider according to recommended ranges given in Si4x6x datasheet  
  _outDiv = 4;
  uint8_t band = 0;

  if (freq < 705000000UL) { _outDiv = 6;  band = 1;};
  if (freq < 525000000UL) { _outDiv = 8;  band = 2;};
  if (freq < 353000000UL) { _outDiv = 12; band = 3;};
  if (freq < 239000000UL) { _outDiv = 16; band = 4;};
  if (freq < 177000000UL) { _outDiv = 24; band = 5;};
  
  _pfdFrequency = 2 * _xtalFrequency / _outDiv;
  
  uint8_t n = ((unsigned int)(freq / _pfdFrequency)) - 1;
  
  float ratio = (float)freq / (float)_pfdFrequency;
  float rest  = ratio - (float)n;
  
  uint32_t m = (unsigned long)(rest * 524288UL); 

  band |= 0x08; // High power setting

  /* set up CLKGEN */
  {
    uint8_t data[] = { 
      0x20, 0x01, 0x51, band };
    sendCommand(SI_CMD_SET_PROPERTY, data, sizeof(data));
  }
    
  // Set the step size
  unsigned char size_1 = 0x00;
  unsigned char size_0 = 0x02;

  /* Set the pll parameters */
  {
    uint8_t data[] = {
      0x40, 0x06, 0x00, 
      n, 
      (uint8_t)(m >> 16), 
      (uint8_t)(m >> 8), 
      (uint8_t)(m), 
      //size_1, size_0
    };
    sendCommand(SI_CMD_SET_PROPERTY, data, sizeof(data));
  }
  
  //setState(kStateTXTune);
}

void Si4x6x::startTX(uint8_t channel, uint16_t pktLength, State txCompleteState)
{
  uint8_t data[] = { 
    channel, 
    (uint8_t)(txCompleteState << 4),
    (uint8_t)(pktLength >> 8), 
    (uint8_t)pktLength 
  };

  /* Change to ready state */
  //setState(kStateReady);

  /* Send START_TX command */
  sendCommand(SI_CMD_START_TX, data, sizeof(data));
}

void Si4x6x::startRX(uint8_t channel, uint16_t pktLength)
{
  uint8_t data[] = { 
    channel, 
    0,
    (uint8_t)(pktLength >> 8), 
    (uint8_t)pktLength,
    0x00,
    0x03,
    0x03
  };

  /* Send START_TX command */
  sendCommand(SI_CMD_START_RX, data, sizeof(data));
}

void Si4x6x::writeTX(const uint8_t *data, uint8_t length)
{
  sendCommand(SI_CMD_WRITE_TX_FIFO, data, length);
}

void Si4x6x::readRX(uint8_t *data, uint8_t length)
{
  sendCommand(SI_CMD_READ_RX_FIFO, 0, 0, data, length);
}

void Si4x6x::configureGPIO(uint8_t gpio0, uint8_t gpio1, uint8_t gpio2, uint8_t gpio3, uint8_t nirq, uint8_t sdo, uint8_t genConfig)
{
  uint8_t data[] = { gpio0, gpio1, gpio2, gpio3, nirq, sdo, genConfig };

  uint8_t reply[7];

  sendCommand(SI_CMD_GPIO_PIN_CFG, data, sizeof(data), reply, 7);
}


void Si4x6x::setPreambleLength(uint8_t length)
{
  uint8_t data[] = { 
    0x10, 0x01, 0x00,
    length
  };

  sendCommand(SI_CMD_SET_PROPERTY, data, sizeof(data));
}


void Si4x6x::setSync(uint8_t config)
{
  uint8_t data[] = { 
    0x11, 0x01, 0x00,
    config
  };

  sendCommand(SI_CMD_SET_PROPERTY, data, sizeof(data));
}


void Si4x6x::setPowerLevel(uint8_t level)
{
  uint8_t data[] = { 
    0x22, 0x01, 0x01,
    level
  };

  sendCommand(SI_CMD_SET_PROPERTY, data, sizeof(data));  
}


void Si4x6x::setNCOModulo(NCOModulo osr) {
  uint32_t ncoFreq;
  
  switch (osr) {
    case kModulo10: ncoFreq = _xtalFrequency / 10; break;
    case kModulo20: ncoFreq = _xtalFrequency / 20; break;
    case kModulo40: ncoFreq = _xtalFrequency / 40; break;
  }
  
  uint8_t data[] = { 
    0x20, 0x04, 0x06,
    (uint8_t)(ncoFreq >> 24) & 0x03 | (uint8_t)(osr << 2),
    (uint8_t)(ncoFreq >> 16),
    (uint8_t)(ncoFreq >> 8),
    (uint8_t)(ncoFreq),
  };
  
  sendCommand(SI_CMD_SET_PROPERTY, data, sizeof(data));
}


void Si4x6x::setDataRate(uint32_t dataRate)
{
  uint8_t data[] = { 
    0x20, 0x03, 0x03,
    (uint8_t)(dataRate >> 16),
    (uint8_t)(dataRate >> 8),
    (uint8_t)(dataRate),
  };
  sendCommand(SI_CMD_SET_PROPERTY, data, sizeof(data));
}


void Si4x6x::setDeviation(uint32_t deviation)
{
  uint32_t x = ((uint64_t)(1ul << 18) * _outDiv * deviation)/ _xtalFrequency;

  uint8_t data[] = { 
    0x20, 0x03, 0x0A,
    (uint8_t)(x >> 16),
    (uint8_t)(x >> 8),
    (uint8_t)(x)
  };
  sendCommand(SI_CMD_SET_PROPERTY, data, 6);
}


void Si4x6x::flushTX()
{
  uint8_t data[] = { 0x01 };
  sendCommand(SI_CMD_FIFO_INFO, data, sizeof(data));
}

void Si4x6x::flushRX()
{
  uint8_t data[] = { 0x02 };
  sendCommand(SI_CMD_FIFO_INFO, data, sizeof(data));
}

uint8_t Si4x6x::getAvailableRX()
{
  uint8_t data[] = { 0x00 };
  uint8_t reply[2];
  sendCommand(SI_CMD_FIFO_INFO, data, sizeof(data), reply, 2);
  return reply[0];
}

void Si4x6x::clearIRQ(void)
{
  uint8_t data[] = { 0x00, 0x00, 0x00 };
  sendCommand(SI_CMD_GET_INT_STATUS, data, sizeof(data));
}

void Si4x6x::clearIRQ(IRQStatus &status)
{
  uint8_t data[] = { 0x00, 0x00, 0x00 };
  sendCommand(SI_CMD_GET_INT_STATUS, data, sizeof(data), status.rawData, 8);
}

void Si4x6x::getModemStatus(ModemStatus &status)
{
  sendCommand(SI_CMD_GET_MODEM_STATUS, 0, 0, status.rawData, 8);
}

void Si4x6x::getChipStatus(ChipStatus &status)
{
  sendCommand(SI_CMD_GET_CHIP_STATUS, 0, 0, status.rawData, 3);
}

void Si4x6x::getPartInfo(PartInfo &info)
{
  sendCommand(SI_CMD_PART_INFO, 0, 0, info.rawData, 8);
}

