// NAME: PN5180.cpp
//
// DESC: Implementation of PN5180 class.
//
// Copyright (c) 2018 by Andreas Trappmann. All rights reserved.
//
// This file is part of the PN5180 library for the Arduino environment.
//
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.
//
// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
// Lesser General Public License for more details.
//
//#define DEBUG 1

#include <Arduino.h>
#include "PN5180.h"
#include "Debug.h"

uint8_t PN5180::readBuffer[508];

PN5180::PN5180(uint8_t SSpin, uint8_t BUSYpin, uint8_t RSTpin, SPIClass& spi) :
  PN5180_NSS(SSpin),
  PN5180_BUSY(BUSYpin),
  PN5180_RST(RSTpin),
  PN5180_SPI(spi)
{
  /*
   * 11.4.1 Physical Host Interface
   * The interface of the PN5180 to a host microcontroller is based on a SPI interface,
   * extended by signal line BUSY. The maximum SPI speed is 7 Mbps and fixed to CPOL
   * = 0 and CPHA = 0.
   */
  // Settings for PN5180: 7Mbps, MSB first, SPI_MODE0 (CPOL=0, CPHA=0)
  SPI_SETTINGS = SPISettings(7000000, MSBFIRST, SPI_MODE0);
}


void PN5180::begin() {
  pinMode(PN5180_NSS, OUTPUT);
  pinMode(PN5180_BUSY, INPUT);
  pinMode(PN5180_RST, OUTPUT);

  digitalWrite(PN5180_NSS, HIGH); // disable
  digitalWrite(PN5180_RST, HIGH); // no reset

  PN5180_SPI.begin();
  PN5180DEBUG(F("SPI pinout: "));
  PN5180DEBUG(F("SS=")); PN5180DEBUG(SS);
  PN5180DEBUG(F(", MOSI=")); PN5180DEBUG(MOSI);
  PN5180DEBUG(F(", MISO=")); PN5180DEBUG(MISO);
  PN5180DEBUG(F(", SCK=")); PN5180DEBUG(SCK);
  PN5180DEBUG("\n");
}

void PN5180::end() {
  digitalWrite(PN5180_NSS, HIGH); // disable
  PN5180_SPI.end();
}

/*
 * WRITE_REGISTER - 0x00
 * This command is used to write a 32-bit value (little endian) to a configuration register.
 * The address of the register must exist. If the condition is not fulfilled, an exception is
 * raised.
 */
bool PN5180::writeRegister(uint8_t reg, uint32_t value) {
  /*
  For all 4 byte command parameter transfers (e.g. register values), the payload
  parameters passed follow the little endian approach (Least Significant Byte first).
   */
  uint8_t *p = (uint8_t*)&value;
  uint8_t buf[6] = { PN5180_WRITE_REGISTER, reg, p[0], p[1], p[2], p[3] };

  PN5180_SPI.beginTransaction(SPI_SETTINGS);
  transceiveCommand(buf, 6);
  PN5180_SPI.endTransaction();

  return true;
}

/*
 * WRITE_REGISTER_OR_MASK - 0x01
 * This command modifies the content of a register using a logical OR operation. The
 * content of the register is read and a logical OR operation is performed with the provided
 * mask. The modified content is written back to the register.
 * The address of the register must exist. If the condition is not fulfilled, an exception is
 * raised.
 */
bool PN5180::writeRegisterWithOrMask(uint8_t reg, uint32_t mask) {
  uint8_t *p = (uint8_t*)&mask;
  uint8_t buf[6] = { PN5180_WRITE_REGISTER_OR_MASK, reg, p[0], p[1], p[2], p[3] };

  PN5180_SPI.beginTransaction(SPI_SETTINGS);
  transceiveCommand(buf, 6);
  PN5180_SPI.endTransaction();

  return true;
}

/*
 * WRITE _REGISTER_AND_MASK - 0x02
 * This command modifies the content of a register using a logical AND operation. The
 * content of the register is read and a logical AND operation is performed with the provided
 * mask. The modified content is written back to the register.
 * The address of the register must exist. If the condition is not fulfilled, an exception is
 * raised.
 */
bool PN5180::writeRegisterWithAndMask(uint8_t reg, uint32_t mask) {
  uint8_t *p = (uint8_t*)&mask;
  uint8_t buf[6] = { PN5180_WRITE_REGISTER_AND_MASK, reg, p[0], p[1], p[2], p[3] };

  PN5180_SPI.beginTransaction(SPI_SETTINGS);
  transceiveCommand(buf, 6);
  PN5180_SPI.endTransaction();

  return true;
}

/*
 * READ_REGISTER - 0x04
 * This command is used to read the content of a configuration register. The content of the
 * register is returned in the 4 byte response.
 * The address of the register must exist. If the condition is not fulfilled, an exception is
 * raised.
 */
bool PN5180::readRegister(uint8_t reg, uint32_t *value) {
  uint8_t cmd[2] = { PN5180_READ_REGISTER, reg };

  PN5180_SPI.beginTransaction(SPI_SETTINGS);
  transceiveCommand(cmd, 2, (uint8_t*)value, 4);
  PN5180_SPI.endTransaction();

#ifdef DEBUG
  switch (reg) {
    case IRQ_STATUS:
      PN5180DEBUG(F("=> IRQ-Status=0x"));
      PN5180DEBUG(formatHex(*value));
      PN5180DEBUG(F(":"));
      if (*value & 0x80000) PN5180DEBUG(" LPCD");
      if (*value & 0x40000) PN5180DEBUG(" HVErr");
      if (*value & 0x20000) PN5180DEBUG(" GeneralErr");
      if (*value & 0x10000) PN5180DEBUG(" TempSensErr");
      if (*value & 0x8000) PN5180DEBUG(" RxSCDet");
      if (*value & 0x4000) PN5180DEBUG(" RxSOFDet");
      if (*value & 0x2000) PN5180DEBUG(" Timer2");
      if (*value & 0x1000) PN5180DEBUG(" Timer1");
      if (*value & 0x800) PN5180DEBUG(" Timer0");
      if (*value & 0x400) PN5180DEBUG(" RFActiveErr");
      if (*value & 0x200) PN5180DEBUG(" TxRFOn");
      if (*value & 0x100) PN5180DEBUG(" TxRFOff");
      if (*value & 0x80) PN5180DEBUG(" RFOnDet");
      if (*value & 0x40) PN5180DEBUG(" RFOffDet");
      if (*value & 0x20) PN5180DEBUG(" StateChange");
      if (*value & 0x10) PN5180DEBUG(" CardAct");
      if (*value & 0x8) PN5180DEBUG(" ModeDet");
      if (*value & 0x4) PN5180DEBUG(" Idle");
      if (*value & 0x2) PN5180DEBUG(" TxDone");
      if (*value & 0x1) PN5180DEBUG(" RxDone");
      PN5180DEBUG("\n");
      break;

    case RF_STATUS: {
      PN5180DEBUG(F("=> RF-Status=0x"));
      PN5180DEBUG(formatHex(*value));
      PN5180DEBUG(F(": "));
      uint8_t xstate = (*value >> 24) & 0x07;
      switch (xstate) {
#define S(s) case PN5180_TS_ ## s: PN5180DEBUG(#s); break;
        S(Idle)
        S(WaitTransmit)
        S(Transmitting)
        S(WaitReceive)
        S(WaitForData)
        S(Receiving)
        S(LoopBack)
        S(RESERVED)
#undef S
        default:
          PN5180DEBUG(F("xstate=0x"));
          PN5180DEBUG(formatHex((uint8_t) xstate));
          break;
      }

      PN5180DEBUG(" DPC=0x");
      PN5180DEBUG(formatHex(uint8_t((*value >> 20) & 0x0F)));
      if (*value & 0x80000) PN5180DEBUG(" DPLL");
      if (*value & 0x40000) PN5180DEBUG(" CRC-ok");
      if (*value & 0x20000) PN5180DEBUG(" TxRF");
      if (*value & 0x10000) PN5180DEBUG(" RFDet");

      switch ((*value >> 13) & 0x07) {
        case 1: PN5180DEBUG(" [Ext RF in TIDT]"); break;
        case 2: PN5180DEBUG(" [Ext RF in TADT]"); break;
        case 3: PN5180DEBUG(" [NO ext RF in TADT]"); break;
        case 4: PN5180DEBUG(" [Ext RF cutoff]"); break;
      }

      if (*value & 0x1000) PN5180DEBUG(" Rx-en");
      if (*value & 0x800) PN5180DEBUG(" Tx-act");
      if (*value & 0x400) PN5180DEBUG(" Rx-act");
      PN5180DEBUG(" AGC=0x");
      PN5180DEBUG(*value & 0x3FF);
      PN5180DEBUG("\n");
      break;
    }

    case RX_STATUS: {
      PN5180DEBUG(F("=> RX-Status=0x"));
      PN5180DEBUG(formatHex(*value));
      PN5180DEBUG(F(":"));

      if (*value & 0x20000) PN5180DEBUG(" ProtoErr");
      if (*value & 0x10000) PN5180DEBUG(" DataErr");
      if (*value & 0x40000) PN5180DEBUG(" CollDet");

      uint8_t coll_pos = (*value >> 19) & 0x7F;
      if (coll_pos) {
        PN5180DEBUG(F(" coll_pos="));
        PN5180DEBUG(coll_pos);
      }

      uint8_t num_frames = (*value >> 9) & 0x0F;
      if (num_frames) {
        PN5180DEBUG(F(" frames="));
        PN5180DEBUG(num_frames);
      }

      uint16_t num_bytes = *value & 0x1FF;
      if (num_bytes) {
        PN5180DEBUG(F(" bytes="));
        PN5180DEBUG(num_bytes);
      }

      uint8_t last_bits = (*value >> 13) & 0x03;
      if (last_bits) {
        PN5180DEBUG(F(" last_bits="));
        PN5180DEBUG(last_bits);
      }
      PN5180DEBUG("\n");
      break;
    }
  }
#endif

  return true;
}

/*
 * WRITE_EEPROM - 0x06
 */
bool PN5180::writeEEprom(uint8_t addr, uint8_t *buffer, uint8_t len) {
  uint8_t cmd[len + 2];
  cmd[0] = PN5180_WRITE_EEPROM;
  cmd[1] = addr;
  for (int i = 0; i < len; i++) cmd[2 + i] = buffer[i];
  PN5180_SPI.beginTransaction(SPI_SETTINGS);
  transceiveCommand(cmd, len + 2);
  PN5180_SPI.endTransaction();
  return true;
}

/*
 * READ_EEPROM - 0x07
 * This command is used to read data from EEPROM memory area. The field 'Address'
 * indicates the start address of the read operation. The field Length indicates the number
 * of bytes to read. The response contains the data read from EEPROM (content of the
 * EEPROM); The data is read in sequentially increasing order starting with the given
 * address.
 * EEPROM Address must be in the range from 0 to 254, inclusive. Read operation must
 * not go beyond EEPROM address 254. If the condition is not fulfilled, an exception is
 * raised.
 */
bool PN5180::readEEprom(uint8_t addr, uint8_t *buffer, int len) {
  if ((addr > 254) || ((addr+len) > 254)) {
    PN5180DEBUG(F("ERROR: Reading beyond addr 254!\n"));
    return false;
  }

  uint8_t cmd[3] = { PN5180_READ_EEPROM, addr, uint8_t(len) };

  PN5180_SPI.beginTransaction(SPI_SETTINGS);
  transceiveCommand(cmd, 3, buffer, len);
  PN5180_SPI.endTransaction();

  return true;
}


/*
 * SEND_DATA - 0x09
 * This command writes data to the RF transmission buffer and starts the RF transmission.
 * The parameter ‘Number of valid bits in last Byte’ indicates the exact number of bits to be
 * transmitted for the last byte (for non-byte aligned frames).
 * Precondition: Host shall configure the Transceiver by setting the register
 * SYSTEM_CONFIG.COMMAND to 0x3 before using the SEND_DATA command, as
 * the command SEND_DATA is only writing data to the transmission buffer and starts the
 * transmission but does not perform any configuration.
 * The size of ‘Tx Data’ field must be in the range from 0 to 260, inclusive (the 0 byte length
 * allows a symbol only transmission when the TX_DATA_ENABLE is cleared).‘Number of
 * valid bits in last Byte’ field must be in the range from 0 to 7. The command must not be
 * called during an ongoing RF transmission. Transceiver must be in ‘WaitTransmit’ state
 * with ‘Transceive’ command set. If the condition is not fulfilled, an exception is raised.
 */
bool PN5180::sendData(uint8_t *data, int len, uint8_t validBits) {
  if (len > 260) {
    PN5180DEBUG(F("ERROR: sendData with more than 260 bytes is not supported!\n"));
    return false;
  }

  uint8_t buffer[len+2];
  buffer[0] = PN5180_SEND_DATA;
  buffer[1] = validBits; // number of valid bits of last byte are transmitted (0 = all bits are transmitted)
  for (int i=0; i<len; i++) {
    buffer[2+i] = data[i];
  }

  writeRegisterWithAndMask(SYSTEM_CONFIG, 0xfffffff8);  // Idle/StopCom Command
  writeRegisterWithOrMask(SYSTEM_CONFIG, 0x00000003);   // Transceive Command
  /*
   * Transceive command; initiates a transceive cycle.
   * Note: Depending on the value of the Initiator bit, a
   * transmission is started or the receiver is enabled
   * Note: The transceive command does not finish
   * automatically. It stays in the transceive cycle until
   * stopped via the IDLE/StopCom command
   */

  PN5180TransceiveStat transceiveState = getTransceiveState();
  if (PN5180_TS_WaitTransmit != transceiveState) {
    PN5180DEBUG(F("*** ERROR: Transceiver not in state WaitTransmit!?\n"));
    return false;
  }

  PN5180_SPI.beginTransaction(SPI_SETTINGS);
  bool success = transceiveCommand(buffer, len+2);
  PN5180_SPI.endTransaction();

  return success;
}

/*
 * READ_DATA - 0x0A
 * This command reads data from the RF reception buffer, after a successful reception.
 * The RX_STATUS register contains the information to verify if the reception had been
 * successful. The data is available within the response of the command. The host controls
 * the number of bytes to be read via the SPI interface.
 * The RF data had been successfully received. In case the instruction is executed without
 * preceding an RF data reception, no exception is raised but the data read back from the
 * reception buffer is invalid. If the condition is not fulfilled, an exception is raised.
 */
uint8_t * PN5180::readData(int len) {
  if (len > 508) {
    Serial.println(F("*** FATAL: Reading more than 508 bytes is not supported!"));
    return 0L;
  }

  uint8_t cmd[2] = { PN5180_READ_DATA, 0x00 };

  PN5180_SPI.beginTransaction(SPI_SETTINGS);
  transceiveCommand(cmd, 2, readBuffer, len);
  PN5180_SPI.endTransaction();

  return readBuffer;
}

bool PN5180::readData(uint8_t len, uint8_t *buffer) {
  if (len > 508) {
    return false;
  }
  uint8_t cmd[2] = { PN5180_READ_DATA, 0x00 };
  PN5180_SPI.beginTransaction(SPI_SETTINGS);
  bool success = transceiveCommand(cmd, 2, buffer, len);
  PN5180_SPI.endTransaction();
  return success;
}

/* prepare LPCD registers */
bool PN5180::prepareLPCD() {
  //=======================================LPCD CONFIG================================================================================
  PN5180DEBUG(F("----------------------------------\n"));
  PN5180DEBUG(F("prepare LPCD...\n"));

  uint8_t data[255];
  uint8_t response[256];
    //1. Set Fieldon time                                           LPCD_FIELD_ON_TIME (0x36)
  uint8_t fieldOn = 0xF0;//0x## -> ##(base 10) x 8μs + 62 μs
  data[0] = fieldOn;
  writeEEprom(0x36, data, 1);
  readEEprom(0x36, response, 1);
  fieldOn = response[0];
  PN5180DEBUG("LPCD-fieldOn time: ");
  PN5180DEBUG(formatHex(fieldOn));
  PN5180DEBUG("\n");

    //2. Set threshold level                                         AGC_LPCD_THRESHOLD @ EEPROM 0x37
  uint8_t threshold = 0x03;
  data[0] = threshold;
  writeEEprom(0x37, data, 1);
  readEEprom(0x37, response, 1);
  threshold = response[0];
  PN5180DEBUG("LPCD-threshold: ");
  PN5180DEBUG(formatHex(threshold));
  PN5180DEBUG("\n");

  //3. Select LPCD mode                                               LPCD_REFVAL_GPO_CONTROL (0x38)
  uint8_t lpcdMode = 0x01; // 1 = LPCD SELF CALIBRATION 
                           // 0 = LPCD AUTO CALIBRATION (this mode does not work, should look more into it, no reason why it shouldn't work)
  data[0] = lpcdMode;
  writeEEprom(0x38, data, 1);
  readEEprom(0x38, response, 1);
  lpcdMode = response[0];
  PN5180DEBUG("lpcdMode: ");
  PN5180DEBUG(formatHex(lpcdMode));
  PN5180DEBUG("\n");
  
  // LPCD_GPO_TOGGLE_BEFORE_FIELD_ON (0x39)
  uint8_t beforeFieldOn = 0xF0; 
  data[0] = beforeFieldOn;
  writeEEprom(0x39, data, 1);
  readEEprom(0x39, response, 1);
  beforeFieldOn = response[0];
  PN5180DEBUG("beforeFieldOn: ");
  PN5180DEBUG(formatHex(beforeFieldOn));
  PN5180DEBUG("\n");
  
  // LPCD_GPO_TOGGLE_AFTER_FIELD_ON (0x3A)
  uint8_t afterFieldOn = 0xF0; 
  data[0] = afterFieldOn;
  writeEEprom(0x3A, data, 1);
  readEEprom(0x3A, response, 1);
  afterFieldOn = response[0];
  PN5180DEBUG("afterFieldOn: ");
  PN5180DEBUG(formatHex(afterFieldOn));
  PN5180DEBUG("\n");
  delay(100);
  return true;
}

/* switch the mode to LPCD (low power card detection)
 * Parameter 'wakeupCounterInMs' must be in the range from 0x0 - 0xA82
 * max. wake-up time is 2960 ms.
 */
bool PN5180::switchToLPCD(uint16_t wakeupCounterInMs) {
  // clear all IRQ flags
  clearIRQStatus(0xffffffff); 
  // enable only LPCD and general error IRQ
  writeRegister(IRQ_ENABLE, LPCD_IRQ_STAT | GENERAL_ERROR_IRQ_STAT);  
  // switch mode to LPCD 
  uint8_t cmd[4] = { PN5180_SWITCH_MODE, 0x01, (uint8_t)(wakeupCounterInMs & 0xFF), (uint8_t)((wakeupCounterInMs >> 8U) & 0xFF) };
  PN5180_SPI.beginTransaction(SPI_SETTINGS);
  bool success = transceiveCommand(cmd, sizeof(cmd));
  PN5180_SPI.endTransaction();
  return success;
}

/*
 * LOAD_RF_CONFIG - 0x11
 * Parameter 'Transmitter Configuration' must be in the range from 0x0 - 0x1C, inclusive. If
 * the transmitter parameter is 0xFF, transmitter configuration is not changed.
 * Field 'Receiver Configuration' must be in the range from 0x80 - 0x9C, inclusive. If the
 * receiver parameter is 0xFF, the receiver configuration is not changed. If the condition is
 * not fulfilled, an exception is raised.
 * The transmitter and receiver configuration shall always be configured for the same
 * transmission/reception speed. No error is returned in case this condition is not taken into
 * account.
 *
 * Transmitter: RF   Protocol          Speed     Receiver: RF    Protocol    Speed
 * configuration                       (kbit/s)  configuration               (kbit/s)
 * byte (hex)                                    byte (hex)
 * ----------------------------------------------------------------------------------------------
 * ->0D              ISO 15693 ASK100  26        8D              ISO 15693   26
 *   0E              ISO 15693 ASK10   26        8E              ISO 15693   53
 */
bool PN5180::loadRFConfig(uint8_t txConf, uint8_t rxConf) {
  uint8_t cmd[3] = { PN5180_LOAD_RF_CONFIG, txConf, rxConf };

  PN5180_SPI.beginTransaction(SPI_SETTINGS);
  transceiveCommand(cmd, 3);
  PN5180_SPI.endTransaction();

  return true;
}

/*
 * RF_ON - 0x16
 * This command is used to switch on the internal RF field. If enabled the TX_RFON_IRQ is
 * set after the field is switched on.
 */
bool PN5180::setRF_on() {
  uint8_t cmd[2] = { PN5180_RF_ON, 0x00 };

  PN5180_SPI.beginTransaction(SPI_SETTINGS);
  transceiveCommand(cmd, 2);
  PN5180_SPI.endTransaction();

  unsigned long startedWaiting = millis();
  while (0 == (TX_RFON_IRQ_STAT & getIRQStatus())) {   // wait for RF field to set up (max 500ms)
    if (millis() - startedWaiting > 500) {
	  PN5180DEBUG(F("Timeout setting RF ON\n"));
	  return false; 
	}
  }; 
  
  clearIRQStatus(TX_RFON_IRQ_STAT);
  return true;
}

/*
 * RF_OFF - 0x17
 * This command is used to switch off the internal RF field. If enabled, the TX_RFOFF_IRQ
 * is set after the field is switched off.
 */
bool PN5180::setRF_off() {
  uint8_t cmd[2] { PN5180_RF_OFF, 0x00 };

  PN5180_SPI.beginTransaction(SPI_SETTINGS);
  transceiveCommand(cmd, 2);
  PN5180_SPI.endTransaction();

  unsigned long startedWaiting = millis();
  while (0 == (TX_RFOFF_IRQ_STAT & getIRQStatus())) {   // wait for RF field to shut down
    if (millis() - startedWaiting > 500) {
	  PN5180DEBUG(F("Timeout setting RF OFF\n"));
	  return false; 
	}
  }; 
  clearIRQStatus(TX_RFOFF_IRQ_STAT);
  return true;
}

//---------------------------------------------------------------------------------------------

/*
11.4.3.1 A Host Interface Command consists of either 1 or 2 SPI frames depending whether the
host wants to write or read data from the PN5180. An SPI Frame consists of multiple
bytes.

All commands are packed into one SPI Frame. An SPI Frame consists of multiple bytes.
No NSS toggles allowed during sending of an SPI frame.

For all 4 byte command parameter transfers (e.g. register values), the payload
parameters passed follow the little endian approach (Least Significant Byte first).

Direct Instructions are built of a command code (1 Byte) and the instruction parameters
(max. 260 bytes). The actual payload size depends on the instruction used.
Responses to direct instructions contain only a payload field (no header).
All instructions are bound to conditions. If at least one of the conditions is not fulfilled, an exception is
raised. In case of an exception, the IRQ line of PN5180 is asserted and corresponding interrupt
status register contain information on the exception.
*/

/*
 * A Host Interface Command consists of either 1 or 2 SPI frames depending whether the
 * host wants to write or read data from the PN5180. An SPI Frame consists of multiple
 * bytes.
 * All commands are packed into one SPI Frame. An SPI Frame consists of multiple bytes.
 * No NSS toggles allowed during sending of an SPI frame.
 * For all 4 byte command parameter transfers (e.g. register values), the payload
 * parameters passed follow the little endian approach (Least Significant Byte first).
 * The BUSY line is used to indicate that the system is BUSY and cannot receive any data
 * from a host. Recommendation for the BUSY line handling by the host:
 * 1. Assert NSS to Low
 * 2. Perform Data Exchange
 * 3. Wait until BUSY is high
 * 4. Deassert NSS
 * 5. Wait until BUSY is low
 * If there is a parameter error, the IRQ is set to ACTIVE and a GENERAL_ERROR_IRQ is set.
 */
bool PN5180::transceiveCommand(uint8_t *sendBuffer, size_t sendBufferLen, uint8_t *recvBuffer, size_t recvBufferLen) {
#ifdef DEBUG
  PN5180DEBUG(F("Sending SPI frame: ["));
  for (uint8_t i=0; i<sendBufferLen; i++) {
    if (i > 0) PN5180DEBUG(" ");

    // Decode commands and register names for debugging
    if (i == 0) {
      switch (sendBuffer[i]) {
#define C(s) case PN5180_ ## s: PN5180DEBUG(#s); break;
        C(WRITE_REGISTER)
        C(WRITE_REGISTER_OR_MASK)
        C(WRITE_REGISTER_AND_MASK)
        C(READ_REGISTER)
        C(WRITE_EEPROM)
        C(READ_EEPROM)
        C(SEND_DATA)
        C(READ_DATA)
        C(SWITCH_MODE)
        C(LOAD_RF_CONFIG)
        C(RF_ON)
        C(RF_OFF)
#undef C
        default: PN5180DEBUG(formatHex(sendBuffer[i])); break;
      }
    } else if (i == 1 && sendBuffer[0] <= PN5180_READ_REGISTER) {
      switch (sendBuffer[i]) {
#define R(s) case s: PN5180DEBUG(#s); break;
        R(SYSTEM_CONFIG);
        R(IRQ_ENABLE);
        R(IRQ_STATUS);
        R(IRQ_CLEAR);
        R(TRANSCEIVE_CONTROL);
        R(TIMER1_RELOAD);
        R(TIMER1_CONFIG);
        R(RX_WAIT_CONFIG);
        R(CRC_RX_CONFIG);
        R(RX_STATUS);
        R(TX_WAIT_CONFIG);
        R(TX_CONFIG);
        R(CRC_TX_CONFIG);
        R(RF_STATUS);
        R(SYSTEM_STATUS);
        R(TEMP_CONTROL);
        R(AGC_REF_CONFIG);
#undef R
        default: PN5180DEBUG(formatHex(sendBuffer[i])); break;
      }
    } else if (i == 1 && sendBuffer[0] <= PN5180_READ_EEPROM) {
      switch (sendBuffer[i]) {
#define E(s) case s: PN5180DEBUG(#s); break;
        E(DIE_IDENTIFIER);
        E(PRODUCT_VERSION);
        E(FIRMWARE_VERSION);
        E(EEPROM_VERSION);
        E(IRQ_PIN_CONFIG);
#undef E
        default: PN5180DEBUG(formatHex(sendBuffer[i])); break;
      }
    } else {
      PN5180DEBUG(formatHex(sendBuffer[i]));
    }
  }
  PN5180DEBUG("]\n");
#endif

  // 0.
  unsigned long startedWaiting = millis();
  while (LOW != digitalRead(PN5180_BUSY)) {
    if (millis() - startedWaiting > commandTimeout) {
		PN5180DEBUG("transceiveCommand timeout (send/0)\n");
		return false;
	};
  }; // wait until busy is low
  // 1.
  digitalWrite(PN5180_NSS, LOW); delay(1);
  // 2.
  PN5180_SPI.transfer((uint8_t*)sendBuffer, sendBufferLen);	
  // 3.
  startedWaiting = millis();
  while (HIGH != digitalRead(PN5180_BUSY)) {
    if (millis() - startedWaiting > commandTimeout) {
		PN5180DEBUG("transceiveCommand timeout (send/3)\n");
		return false;
	}
  }; // wait until busy is high
  // 4.
  digitalWrite(PN5180_NSS, HIGH); delay(1);
  // 5.
  startedWaiting = millis();
  while (LOW != digitalRead(PN5180_BUSY)) {
    if (millis() - startedWaiting > commandTimeout) {
		PN5180DEBUG("transceiveCommand timeout (send/5)\n");
		return false;
	};
  }; // wait until busy is low

  // check, if write-only
  if ((0 == recvBuffer) || (0 == recvBufferLen)) return true;
  PN5180DEBUG(F("Receive SPI frame: "));

  // 1.
  digitalWrite(PN5180_NSS, LOW); 
  // 2.
  memset(recvBuffer, 0xFF, recvBufferLen);
  PN5180_SPI.transfer(recvBuffer, recvBufferLen);
  // 3.
  startedWaiting = millis(); //delay(1);
  while (HIGH != digitalRead(PN5180_BUSY)) {
    if (millis() - startedWaiting > commandTimeout) {
		PN5180DEBUG("transceiveCommand timeout (receive/3)\n");
		return false;
	};
  }; // wait until busy is high
  // 4.
  digitalWrite(PN5180_NSS, HIGH); 
  // 5.
  startedWaiting = millis();
  while (LOW != digitalRead(PN5180_BUSY)) {
    if (millis() - startedWaiting > commandTimeout) {
		PN5180DEBUG("transceiveCommand timeout (receive/5)\n");
		return false;
	};
  }; // wait until busy is low

#ifdef DEBUG
  PN5180DEBUG(F("["));
  for (uint8_t i=0; i<recvBufferLen; i++) {
    if (i > 0) PN5180DEBUG(" ");
    PN5180DEBUG(formatHex(recvBuffer[i]));
  }
  PN5180DEBUG("]\n");
#endif

  return true;
}

/*
 * Reset NFC device
 */
void PN5180::reset() {
  PN5180DEBUG(F("Resetting PN5180...\n"));
  digitalWrite(PN5180_RST, LOW);  // at least 10us required
  delay(1);
  digitalWrite(PN5180_RST, HIGH); // 2ms to ramp up required
  delay(5);

  unsigned long startedWaiting = millis();
  while (0 == (IDLE_IRQ_STAT & getIRQStatus())) {
	// wait for system to start up (with timeout)
    if (millis() - startedWaiting > commandTimeout) {
		PN5180DEBUG(F("reset failed (timeout)!!!\n"));
		// try again with larger time
		digitalWrite(PN5180_RST, LOW);  
		delay(10);
		digitalWrite(PN5180_RST, HIGH); 
		delay(50);
		return;
	}
  }
}

/**
 * @name  getInterrupt
 * @desc  read interrupt status register and clear interrupt status
 */
uint32_t PN5180::getIRQStatus() {
  uint32_t irqStatus;
  readRegister(IRQ_STATUS, &irqStatus);
  return irqStatus;
}

bool PN5180::clearIRQStatus(uint32_t irqMask) {
  return writeRegister(IRQ_CLEAR, irqMask);
}

/*
 * Get TRANSCEIVE_STATE from RF_STATUS register
 */
PN5180TransceiveStat PN5180::getTransceiveState() {
  uint32_t rfStatus;
  if (!readRegister(RF_STATUS, &rfStatus)) {
    PN5180DEBUG(F("ERROR reading RF_STATUS register.\n"));
    return PN5180TransceiveStat(0);
  }

  return PN5180TransceiveStat((rfStatus >> 24) & 0x07);
}
