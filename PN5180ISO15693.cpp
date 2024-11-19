// NAME: PN5180ISO15693.h
//
// DESC: ISO15693 protocol on NXP Semiconductors PN5180 module for Arduino.
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
#include "PN5180ISO15693.h"
#include "Debug.h"

PN5180ISO15693::PN5180ISO15693(uint8_t SSpin, uint8_t BUSYpin, uint8_t RSTpin, SPIClass& spi) 
              : PN5180(SSpin, BUSYpin, RSTpin, spi) {
}

/*
 * Inventory, code=01
 *
 * Request format: SOF, Req.Flags, Inventory, AFI (opt.), Mask len, Mask value, CRC16, EOF
 * Response format: SOF, Resp.Flags, DSFID, UID, CRC16, EOF
 *
 */
ISO15693ErrorCode PN5180ISO15693::getInventory(uint8_t *uid) {
  //                     Flags,  CMD, maskLen
  uint8_t inventory[] = { 0x26, 0x01, 0x00 };
  //                        |\- inventory flag + high data rate
  //                        \-- 1 slot: only one card, no AFI field present

  for (int i=0; i<8; i++) {
    uid[i] = 0;  
  }
  
  uint8_t *readBuffer;
  ISO15693ErrorCode rc = issueISO15693Command(inventory, sizeof(inventory), &readBuffer);
  if (ISO15693_EC_OK != rc) {
    return rc;
  }

  PN5180DEBUG(F("Response flags: "));
  PN5180DEBUG(formatHex(readBuffer[0]));
  PN5180DEBUG(F(", Data Storage Format ID: "));
  PN5180DEBUG(formatHex(readBuffer[1]));
  PN5180DEBUG(F(", UID: "));
  
  for (int i=0; i<8; i++) {
    uid[i] = readBuffer[2+i];
#ifdef DEBUG
    PN5180DEBUG(formatHex(uid[7-i])); // LSB comes first
    if (i<2) PN5180DEBUG(":");
#endif
  }
  
  PN5180DEBUG("\n");

  return ISO15693_EC_OK;
}

/*
 * Inventory with flag set for 16 time slots, code=01
 * https://www.nxp.com.cn/docs/en/application-note/AN12650.pdf
 * Request format: SOF, Req.Flags, Inventory, AFI (opt.), Mask len, Mask value, CRC16, EOF
 * Response format: SOF, Resp.Flags, DSFID, UID, CRC16, EOF
 */
ISO15693ErrorCode PN5180ISO15693::getInventoryMultiple(uint8_t *uid, uint8_t maxTags, uint8_t *numCard) {
  PN5180DEBUG("PN5180ISO15693: Get Inventory...");
  uint16_t collision[maxTags];
  *numCard = 0;
  uint8_t numCollisions = 0;
  // Send an inventory command and listen for the response
  inventoryPoll(uid, maxTags, numCard, &numCollisions, collision);
  PN5180DEBUG("Number of collisions=");
  PN5180DEBUG(numCollisions);
  PN5180DEBUG("\n");
	// Continue to call inventory until no further collisions detected
	// (numCard will be incremented automatically on each call) 
  while(numCollisions > 0){                                                 
#ifdef DEBUG
    printf("inventoryPoll: Polling with mask=0x%X\n", collision[0]);
#endif
    inventoryPoll(uid, maxTags, numCard, &numCollisions, collision);
    numCollisions--;
    for(int i=0; i<numCollisions; i++){
      collision[i] = collision[i+1];
    }
  }
  return ISO15693_EC_OK;
}

/**
 * https://www.nxp.com.cn/docs/en/application-note/AN12650.pdf
 * 4.2.1 Example Code and 4.2.2 Description
 */
ISO15693ErrorCode PN5180ISO15693::inventoryPoll(uint8_t *uid, uint8_t maxTags, uint8_t *numCard, uint8_t *numCol, uint16_t *collision){
  uint8_t maskLen = 0;
  if(*numCol > 0){
    uint32_t mask = collision[0];
    do{
      mask >>= 4L;
      maskLen++;
    }while(mask > 0);
  } 
  uint8_t *p = (uint8_t*)&(collision[0]);
  //                      Flags,  CMD,
  uint8_t inventory[7] = { 0x06, 0x01, maskLen*4, p[0], p[1], p[2], p[3] };
  //                         |\- inventory flag + high data rate
  //                         \-- 16 slots: upto 16 cards, no AFI field present
  uint8_t cmdLen = 3 + (maskLen/2) + (maskLen%2);
#ifdef DEBUG
  printf("inventoryPoll inputs: maxTags=%d, numCard=%d, numCol=%d\n", maxTags, *numCard, *numCol);
  printf("mask=%d, maskLen=%d, cmdLen=%d\n", p[0], maskLen, cmdLen);
#endif
  clearIRQStatus(0x000FFFFF);                                      // 3. Clear all IRQ_STATUS flags
  sendData(inventory, cmdLen, 0);                                  // 4. 5. 6. Idle/StopCom Command, Transceive Command, Inventory command

  for(int slot=0; slot<16; slot++){                                // 7. Loop to check 16 time slots for data
    uint32_t rxStatus;
    uint32_t irqStatus = getIRQStatus();
    readRegister(RX_STATUS, &rxStatus);
    uint16_t len = (uint16_t)(rxStatus & 0x000001ff);
    if((rxStatus >> 18) & 0x01 && *numCol < maxTags){              // 7+ Determine if a collision occurred
      if(maskLen > 0) collision[*numCol] = collision[0] | (slot << (maskLen * 2));
      else collision[*numCol] = slot << (maskLen * 2); // Yes, store position of collision
      *numCol = *numCol + 1;
#ifdef DEBUG
      printf("Collision detected for UIDs matching %X starting at LSB", collision[*numCol-1]);
#endif
    }
    else if(!(irqStatus & RX_IRQ_STAT) && !len){                   // 8. Check if a card has responded
      PN5180DEBUG("getInventoryMultiple: No card in this time slot. State=");
      PN5180DEBUG(irqStatus);
      PN5180DEBUG("\n");
    }
    else{
#ifdef DEBUG
      printf("slot=%d, irqStatus: %ld, RX_STATUS: %ld, Response length=%d\n", slot, irqStatus, rxStatus, len);
#endif
      uint8_t *readBuffer;
      readBuffer = readData(len+1);                                // 9. Read reception buffer
#ifdef DEBUG
      printf("readBuffer= ");
      for(int i=0; i<len+1; i++){
        if(readBuffer[i]<16) printf("0");
        printf("%X", readBuffer[i]);
        printf(":");
      }
      printf("\n");
#endif
      if(0L == readBuffer){
        PN5180DEBUG("getInventoryMultiple: ERROR in readData!");
        return ISO15693_EC_UNKNOWN_ERROR;
      }

      // Record raw UID data                                       // 10. Record all data to Inventory struct
      for (int i=0; i<8; i++) {
        uint8_t startAddr = (*numCard * 8) + i;
        uid[startAddr] = readBuffer[2+i];
      }
      *numCard = *numCard + 1;

#ifdef DEBUG
      printf("getInventoryMultiple: Response flags: 0x%X, Data Storage Format ID: 0x%X\n", readBuffer[0], readBuffer[1]);
      printf("numCard=%d\n", *numCard);
#endif
    }

    if(slot+1 < 16){ // If we have more cards to poll for...
      writeRegisterWithAndMask(TX_CONFIG, 0xFFFFFB3F);             // 11. Next SEND_DATA will only include EOF
      clearIRQStatus(0x000FFFFF);                                  // 14. Clear all IRQ_STATUS flags
      sendData(inventory, 0, 0);                                   // 12. 13. 15. Idle/StopCom Command, Transceive Command, Send EOF
    }
  }
  setRF_off();                                                     // 16. Switch off RF field
  setupRF();                                                       // 1. 2. Load ISO15693 config, RF on
  return ISO15693_EC_OK;
}

/*
 * Read single block, code=20
 *
 * Request format: SOF, Req.Flags, ReadSingleBlock, UID (opt.), BlockNumber, CRC16, EOF
 * Response format:
 *  when ERROR flag is set:
 *    SOF, Resp.Flags, ErrorCode, CRC16, EOF
 *
 *     Response Flags:
  *    xxxx.3xx0
  *         |||\_ Error flag: 0=no error, 1=error detected, see error field
  *         \____ Extension flag: 0=no extension, 1=protocol format is extended
  *
  *  If Error flag is set, the following error codes are defined:
  *    01 = The command is not supported, i.e. the request code is not recognized.
  *    02 = The command is not recognized, i.e. a format error occurred.
  *    03 = The option is not supported.
  *    0F = Unknown error.
  *    10 = The specific block is not available.
  *    11 = The specific block is already locked and cannot be locked again.
  *    12 = The specific block is locked and cannot be changed.
  *    13 = The specific block was not successfully programmed.
  *    14 = The specific block was not successfully locked.
  *    A0-DF = Custom command error codes
 *
 *  when ERROR flag is NOT set:
 *    SOF, Flags, BlockData (len=blockLength), CRC16, EOF
 */
ISO15693ErrorCode PN5180ISO15693::readSingleBlock(uint8_t *uid, uint8_t blockNo, uint8_t *blockData, uint8_t blockSize) {
  //                            flags, cmd, uid,             blockNo
  uint8_t readSingleBlock[] = { 0x22, 0x20, 1,2,3,4,5,6,7,8, blockNo }; // UID has LSB first!
  //                              |\- high data rate
  //                              \-- no options, addressed by UID
  for (int i=0; i<8; i++) {
    readSingleBlock[2+i] = uid[i];
  }

  uint8_t *resultPtr;
  ISO15693ErrorCode rc = issueISO15693Command(readSingleBlock, sizeof(readSingleBlock), &resultPtr);
  if (ISO15693_EC_OK != rc) {
    return rc;
  }

  PN5180DEBUG("Value=");
  
  for (int i=0; i<blockSize; i++) {
    blockData[i] = resultPtr[1+i];
#ifdef DEBUG    
    PN5180DEBUG(formatHex(blockData[i]));
    PN5180DEBUG(" ");
#endif    
  }

#ifdef DEBUG
  PN5180DEBUG(" ");
  for (int i=0; i<blockSize; i++) {
    char c = blockData[i];
    if (isPrintable(c)) {
      PN5180DEBUG(c);
    }
    else PN5180DEBUG(".");
  }
  PN5180DEBUG("\n");
#endif

  return ISO15693_EC_OK;
}

/*
 * Write single block, code=21
 *
 * Request format: SOF, Requ.Flags, WriteSingleBlock, UID (opt.), BlockNumber, BlockData (len=blcokLength), CRC16, EOF
 * Response format:
 *  when ERROR flag is set:
 *    SOF, Resp.Flags, ErrorCode, CRC16, EOF
 *
 *     Response Flags:
  *    xxxx.3xx0
  *         |||\_ Error flag: 0=no error, 1=error detected, see error field
  *         \____ Extension flag: 0=no extension, 1=protocol format is extended
  *
  *  If Error flag is set, the following error codes are defined:
  *    01 = The command is not supported, i.e. the request code is not recognized.
  *    02 = The command is not recognized, i.e. a format error occurred.
  *    03 = The option is not supported.
  *    0F = Unknown error.
  *    10 = The specific block is not available.
  *    11 = The specific block is already locked and cannot be locked again.
  *    12 = The specific block is locked and cannot be changed.
  *    13 = The specific block was not successfully programmed.
  *    14 = The specific block was not successfully locked.
  *    A0-DF = Custom command error codes
 *
 *  when ERROR flag is NOT set:
 *    SOF, Resp.Flags, CRC16, EOF
 */
ISO15693ErrorCode PN5180ISO15693::writeSingleBlock(uint8_t *uid, uint8_t blockNo, uint8_t *blockData, uint8_t blockSize) {
  //                            flags, cmd, uid,             blockNo
  uint8_t writeSingleBlock[] = { 0x22, 0x21, 1,2,3,4,5,6,7,8, blockNo }; // UID has LSB first!
  //                               |\- high data rate
  //                               \-- no options, addressed by UID

  uint8_t writeCmdSize = sizeof(writeSingleBlock) + blockSize;
  uint8_t *writeCmd = (uint8_t*)malloc(writeCmdSize);
  uint8_t pos = 0;
  writeCmd[pos++] = writeSingleBlock[0];
  writeCmd[pos++] = writeSingleBlock[1];
  for (int i=0; i<8; i++) {
    writeCmd[pos++] = uid[i];
  }
  writeCmd[pos++] = blockNo;
  for (int i=0; i<blockSize; i++) {
    writeCmd[pos++] = blockData[i];
  }

  uint8_t *resultPtr;
  ISO15693ErrorCode rc = issueISO15693Command(writeCmd, writeCmdSize, &resultPtr);
  if (ISO15693_EC_OK != rc) {
    free(writeCmd);
    return rc;
  }

  free(writeCmd);
  return ISO15693_EC_OK;
}

/*
 * Read multiple block, code=23
 *
 * Request format: SOF, Req.Flags, ReadMultipleBlock, UID (opt.), FirstBlockNumber, numBlocks, CRC16, EOF
 * Response format:
 *  when ERROR flag is set:
 *    SOF, Resp.Flags, ErrorCode, CRC16, EOF
 *
 *     Response Flags:
  *    xxxx.3xx0
  *         |||\_ Error flag: 0=no error, 1=error detected, see error field
  *         \____ Extension flag: 0=no extension, 1=protocol format is extended
  *
  *  If Error flag is set, the following error codes are defined:
  *    01 = The command is not supported, i.e. the request code is not recognized.
  *    02 = The command is not recognized, i.e. a format error occurred.
  *    03 = The option is not supported.
  *    0F = Unknown error.
  *    10 = The specific block is not available.
  *    11 = The specific block is already locked and cannot be locked again.
  *    12 = The specific block is locked and cannot be changed.
  *    13 = The specific block was not successfully programmed.
  *    14 = The specific block was not successfully locked.
  *    A0-DF = Custom command error codes
 *
 *  when ERROR flag is NOT set:
 *    SOF, Flags, BlockData (len=blockSize * numBlock), CRC16, EOF
 */
ISO15693ErrorCode PN5180ISO15693::readMultipleBlock(uint8_t *uid, uint8_t blockNo, uint8_t numBlock, uint8_t *blockData, uint8_t blockSize) {
  if(blockNo > numBlock-1){ // Attempted to start at a block greater than the num blocks on the VICC
    PN5180DEBUG("Starting block exceeds length of data");
    return ISO15693_EC_BLOCK_NOT_AVAILABLE;
  }
  if( (blockNo + numBlock) > numBlock ){ // Will attempt to read a block greater than the num blocks on the VICC 
    PN5180DEBUG("End of block exceeds length of data");
    return ISO15693_EC_BLOCK_NOT_AVAILABLE;
  }

  //                              flags, cmd, uid,             1stBlock blocksToRead  
  uint8_t readMultipleCmd[12] = { 0x22, 0x23, 1,2,3,4,5,6,7,8, blockNo, numBlock-1 }; // UID has LSB first!
  //                                |\- high data rate
  //                                \-- no options, addressed by UID

  for (int i=0; i<8; i++) {
    readMultipleCmd[2+i] = uid[i];
  }

  uint8_t *resultPtr;
  ISO15693ErrorCode rc = issueISO15693Command(readMultipleCmd, sizeof(readMultipleCmd), &resultPtr);
  if (ISO15693_EC_OK != rc) return rc;

  PN5180DEBUG("readMultipleBlock: Value=");
  for (int i=0; i<numBlock * blockSize; i++) {
    blockData[i] = resultPtr[1+i];
#ifdef DEBUG    
    PN5180DEBUG(formatHex(blockData[i]));
    PN5180DEBUG(" ");
#endif 
  }

#ifdef DEBUG
  PN5180DEBUG(" ");
  for (int i=0; i<blockSize; i++) {
    char c = blockData[i];
    if (isPrintable(c)) {
      PN5180DEBUG(c);
    }
    else PN5180DEBUG(".");
  }
  PN5180DEBUG("\n");
#endif

  return ISO15693_EC_OK;
}


/*
 * Get System Information, code=2B
 *
 * Request format: SOF, Req.Flags, GetSysInfo, UID (opt.), CRC16, EOF
 * Response format:
 *  when ERROR flag is set:
 *    SOF, Resp.Flags, ErrorCode, CRC16, EOF
 *
 *     Response Flags:
  *    xxxx.3xx0
  *         |||\_ Error flag: 0=no error, 1=error detected, see error field
  *         \____ Extension flag: 0=no extension, 1=protocol format is extended
  *
  *  If Error flag is set, the following error codes are defined:
  *    01 = The command is not supported, i.e. the request code is not recognized.
  *    02 = The command is not recognized, i.e. a format error occurred.
  *    03 = The option is not supported.
  *    0F = Unknown error.
  *    10 = The specific block is not available.
  *    11 = The specific block is already locked and cannot be locked again.
  *    12 = The specific block is locked and cannot be changed.
  *    13 = The specific block was not successfully programmed.
  *    14 = The specific block was not successfully locked.
  *    A0-DF = Custom command error codes
  *
 *  when ERROR flag is NOT set:
 *    SOF, Flags, InfoFlags, UID, DSFID (opt.), AFI (opt.), Other fields (opt.), CRC16, EOF
 *
 *    InfoFlags:
 *    xxxx.3210
 *         |||\_ DSFID: 0=DSFID not supported, DSFID field NOT present; 1=DSFID supported, DSFID field present
 *         ||\__ AFI: 0=AFI not supported, AFI field not present; 1=AFI supported, AFI field present
 *         |\___ VICC memory size:
 *         |        0=Information on VICC memory size is not supported. Memory size field is present. ???
 *         |        1=Information on VICC memory size is supported. Memory size field is present.
 *         \____ IC reference:
 *                  0=Information on IC reference is not supported. IC reference field is not present.
 *                  1=Information on IC reference is supported. IC reference field is not present.
 *
 *    VICC memory size:
 *      xxxb.bbbb nnnn.nnnn
 *        bbbbb - Block size is expressed in number of bytes, on 5 bits, allowing to specify up to 32 bytes i.e. 256 bits.
 *        nnnn.nnnn - Number of blocks is on 8 bits, allowing to specify up to 256 blocks.
 *
 *    IC reference: The IC reference is on 8 bits and its meaning is defined by the IC manufacturer.
 */
ISO15693ErrorCode PN5180ISO15693::getSystemInfo(uint8_t *uid, uint8_t *blockSize, uint8_t *numBlocks) {
  uint8_t sysInfo[] = { 0x22, 0x2b, 1,2,3,4,5,6,7,8 };  // UID has LSB first!
  for (int i=0; i<8; i++) {
    sysInfo[2+i] = uid[i];
  }

  uint8_t *readBuffer;
  ISO15693ErrorCode rc = issueISO15693Command(sysInfo, sizeof(sysInfo), &readBuffer);
  if (ISO15693_EC_OK != rc) {
    return rc;
  }

  for (int i=0; i<8; i++) {
    uid[i] = readBuffer[2+i];
  }
  
#ifdef DEBUG
  PN5180DEBUG("UID=");
  for (int i=0; i<8; i++) {
    PN5180DEBUG(formatHex(readBuffer[9-i]));  // UID has LSB first!
    if (i<2) PN5180DEBUG(":");
  }
  PN5180DEBUG("\n");
#endif
  
  uint8_t *p = &readBuffer[10];

  uint8_t infoFlags = readBuffer[1];
  if (infoFlags & 0x01) { // DSFID flag
    PN5180DEBUG("DSFID=");  // Data storage format identifier
    PN5180DEBUG(formatHex(uint8_t(*p++)));
    PN5180DEBUG("\n");
  }
#ifdef DEBUG
  else PN5180DEBUG(F("No DSFID\n"));  
#endif
  
  if (infoFlags & 0x02) { // AFI flag
    uint8_t afi = *p++;
    PN5180DEBUG(F("AFI="));  // Application family identifier
    PN5180DEBUG(formatHex(afi));
    PN5180DEBUG(F(" - "));
    switch (afi >> 4) {
      case 0: PN5180DEBUG(F("All families")); break;
      case 1: PN5180DEBUG(F("Transport")); break;
      case 2: PN5180DEBUG(F("Financial")); break;
      case 3: PN5180DEBUG(F("Identification")); break;
      case 4: PN5180DEBUG(F("Telecommunication")); break;
      case 5: PN5180DEBUG(F("Medical")); break;
      case 6: PN5180DEBUG(F("Multimedia")); break;
      case 7: PN5180DEBUG(F("Gaming")); break;
      case 8: PN5180DEBUG(F("Data storage")); break;
      case 9: PN5180DEBUG(F("Item management")); break;
      case 10: PN5180DEBUG(F("Express parcels")); break;
      case 11: PN5180DEBUG(F("Postal services")); break;
      case 12: PN5180DEBUG(F("Airline bags")); break;
      default: PN5180DEBUG(F("Unknown")); break;
    }
    PN5180DEBUG("\n");
  }
#ifdef DEBUG
  else PN5180DEBUG(F("No AFI\n"));
#endif

  if (infoFlags & 0x04) { // VICC Memory size
    *numBlocks = *p++;
    *blockSize = *p++;
    *blockSize = (*blockSize) & 0x1f;

    *blockSize = *blockSize + 1; // range: 1-32
    *numBlocks = *numBlocks + 1; // range: 1-256

    PN5180DEBUG("VICC MemSize=");
    PN5180DEBUG(uint16_t(*blockSize) * (*numBlocks));
    PN5180DEBUG(" BlockSize=");
    PN5180DEBUG(*blockSize);
    PN5180DEBUG(" NumBlocks=");
    PN5180DEBUG(*numBlocks);
    PN5180DEBUG("\n");
  }
#ifdef DEBUG
  else PN5180DEBUG(F("No VICC memory size\n"));
#endif
   
  if (infoFlags & 0x08) { // IC reference
    PN5180DEBUG("IC Ref=");
    PN5180DEBUG(formatHex(uint8_t(*p++)));
    PN5180DEBUG("\n");
  }
#ifdef DEBUG
  else PN5180DEBUG(F("No IC ref\n"));
#endif

  return ISO15693_EC_OK;
}


// ICODE SLIX specific commands

/*
 * The GET RANDOM NUMBER command is required to receive a random number from the label IC. 
 * The passwords that will be transmitted with the SET PASSWORD,ENABLEPRIVACY and DESTROY commands 
 * have to be calculated with the password and the random number (see Section 9.5.3.2 "SET PASSWORD")
 */
ISO15693ErrorCode PN5180ISO15693::getRandomNumber(uint8_t *randomData) {
  uint8_t getrandom[] = {0x02, 0xB2, 0x04};
  uint8_t *readBuffer;
  ISO15693ErrorCode rc = issueISO15693Command(getrandom, sizeof(getrandom), &readBuffer);
  if (rc == ISO15693_EC_OK) {
    randomData[0] = readBuffer[1];
    randomData[1] = readBuffer[2];
  }
  return rc;
}

/*
 * The SET PASSWORD command enables the different passwords to be transmitted to the label 
 * to access the different protected functionalities of the following commands. 
 * The SET PASSWORD command has to be executed just once for the related passwords if the label is powered
 */
ISO15693ErrorCode PN5180ISO15693::setPassword(uint8_t identifier, uint8_t *password, uint8_t *random) {
  uint8_t setPassword[] = {0x02, 0xB3, 0x04, 0x04, 0x00, 0x00, 0x00, 0x00};
  uint8_t *readBuffer;
  setPassword[3] = identifier;
  setPassword[4] = password[0] ^ random[0];
  setPassword[5] = password[1] ^ random[1];
  setPassword[6] = password[2] ^ random[0];
  setPassword[7] = password[3] ^ random[1];
  ISO15693ErrorCode rc = issueISO15693Command(setPassword, sizeof(setPassword), &readBuffer);
  return rc;
}

/*
 * The ENABLE PRIVACY command enables the ICODE SLIX2 Label IC to be set to
 * Privacy mode if the Privacy password is correct. The ICODE SLIX2 will not respond to
 * any command except GET RANDOM NUMBER and SET PASSWORD
 */
ISO15693ErrorCode PN5180ISO15693::enablePrivacy(uint8_t *password, uint8_t *random) {
  uint8_t setPrivacy[] = {0x02, 0xBA, 0x04, 0x00, 0x00, 0x00, 0x00};
  uint8_t *readBuffer;
  setPrivacy[3] = password[0] ^ random[0];
  setPrivacy[4] = password[1] ^ random[1];
  setPrivacy[5] = password[2] ^ random[0];
  setPrivacy[6] = password[3] ^ random[1];
  ISO15693ErrorCode rc = issueISO15693Command(setPrivacy, sizeof(setPrivacy), &readBuffer);
  return rc;
}


// disable privacy mode for ICODE SLIX2 tag with given password
ISO15693ErrorCode PN5180ISO15693::disablePrivacyMode(uint8_t *password) {
  // get a random number from the tag
  uint8_t random[]= {0x00, 0x00};
  ISO15693ErrorCode rc = getRandomNumber(random);
  if (rc != ISO15693_EC_OK) {
    return rc;
  }
  
  // set password to disable privacy mode 
  rc = setPassword(0x04, password, random);
  return rc; 
}

// enable privacy mode for ICODE SLIX2 tag with given password 
ISO15693ErrorCode PN5180ISO15693::enablePrivacyMode(uint8_t *password) {
  // get a random number from the tag
  uint8_t random[]= {0x00, 0x00};
  ISO15693ErrorCode rc = getRandomNumber(random);
  if (rc != ISO15693_EC_OK) {
    return rc;
  }
  
  // enable privacy command to lock the tag
  rc = enablePrivacy(password, random);
  return rc; 
}


/*
 * ISO 15693 - Protocol
 *
 * General Request Format:
 *  SOF, Req.Flags, Command code, Parameters, Data, CRC16, EOF
 *
 *  Request Flags:
 *    xxxx.3210
 *         |||\_ Subcarrier flag: 0=single sub-carrier, 1=two sub-carrier
 *         ||\__ Datarate flag: 0=low data rate, 1=high data rate
 *         |\___ Inventory flag: 0=no inventory, 1=inventory
 *         \____ Protocol extension flag: 0=no extension, 1=protocol format is extended
 *
 *  If Inventory flag is set:
 *    7654.xxxx
 *     ||\_ AFI flag: 0=no AFI field present, 1=AFI field is present
 *     |\__ Number of slots flag: 0=16 slots, 1=1 slot
 *     \___ Option flag: 0=default, 1=meaning is defined by command description
 *
 *  If Inventory flag is NOT set:
 *    7654.xxxx
 *     ||\_ Select flag: 0=request shall be executed by any VICC according to Address_flag
 *     ||                1=request shall be executed only by VICC in selected state
 *     |\__ Address flag: 0=request is not addressed. UID field is not present.
 *     |                  1=request is addressed. UID field is present. Only VICC with UID shall answer
 *     \___ Option flag: 0=default, 1=meaning is defined by command description
 *
 * General Response Format:
 *  SOF, Resp.Flags, Parameters, Data, CRC16, EOF
 *
 *  Response Flags:
 *    xxxx.3210
 *         |||\_ Error flag: 0=no error, 1=error detected, see error field
 *         ||\__ RFU: 0
 *         |\___ RFU: 0
 *         \____ Extension flag: 0=no extension, 1=protocol format is extended
 *
 *  If Error flag is set, the following error codes are defined:
 *    01 = The command is not supported, i.e. the request code is not recognized.
 *    02 = The command is not recognized, i.e. a format error occurred.
 *    03 = The option is not supported.
 *    0F = Unknown error.
 *    10 = The specific block is not available.
 *    11 = The specific block is already locked and cannot be locked again.
 *    12 = The specific block is locked and cannot be changed.
 *    13 = The specific block was not successfully programmed.
 *    14 = The specific block was not successfully locked.
 *    A0-DF = Custom command error codes
 *
 *  Function return values:
 *    0 = OK
 *   -1 = No card detected
 *   >0 = Error code
 */
ISO15693ErrorCode PN5180ISO15693::issueISO15693Command(uint8_t *cmd, uint8_t cmdLen, uint8_t **resultPtr) {
#ifdef DEBUG
  PN5180DEBUG(F("Sending ISO15693 ["));
  for (int b = 0; b < cmdLen; b++) {
    if (b > 0) PN5180DEBUG(" ");
    if (b == 0) {
      PN5180DEBUG(F("("));
      PN5180DEBUG((cmd[b] & 0x1) ? F("2Sub") : F("1Sub"));
      PN5180DEBUG((cmd[b] & 0x2) ? F(" HiRate") : F(" LoRate"));
      if (cmd[b] & 0x8) PN5180DEBUG(F(" Ext"));
      if (cmd[b] & 0x4) {
        PN5180DEBUG(F(" Inv"));
        if (cmd[b] & 0x10) PN5180DEBUG(F(" AFI"));
        PN5180DEBUG((cmd[b] & 0x20) ? F(" 1Slot") : F(" 16Slot"));
        if (cmd[b] & 0x40) PN5180DEBUG(F(" Opt"));
      } else {
        if (cmd[b] & 0x10) PN5180DEBUG(F(" Sel"));
        if (cmd[b] & 0x20) PN5180DEBUG(F(" Addr"));
        if (cmd[b] & 0x40) PN5180DEBUG(F(" Opt"));
      }
      PN5180DEBUG(F(")"));
    } else if (b == 1) {
      switch (cmd[b]) {
        case 0x01: PN5180DEBUG(F("INVENTORY")); break;
        case 0x02: PN5180DEBUG(F("STAY-QUIET")); break;
        case 0x20: PN5180DEBUG(F("READ-BLOCK")); break;
        case 0x21: PN5180DEBUG(F("WRITE-BLOCK")); break;
        case 0x22: PN5180DEBUG(F("LOCK-BLOCK")); break;
        case 0x23: PN5180DEBUG(F("READ-BLOCKS")); break;
        case 0x24: PN5180DEBUG(F("WRITE-BLOCKS")); break;
        case 0x25: PN5180DEBUG(F("SELECT")); break;
        case 0x26: PN5180DEBUG(F("RESET-TO-READY")); break;
        case 0x27: PN5180DEBUG(F("WRITE-AFI")); break;
        case 0x28: PN5180DEBUG(F("LOCK-AFI")); break;
        case 0x29: PN5180DEBUG(F("WRITE-DSFID")); break;
        case 0x2A: PN5180DEBUG(F("LOCK-DSFID")); break;
        case 0x2B: PN5180DEBUG(F("GET-SYS-INFO")); break;
        // ICODE SLIX specific commands
        case 0xA0: PN5180DEBUG(F("GET-SECURITY")); break;
        case 0xB2: PN5180DEBUG(F("GET-RANDOM")); break;
        case 0xB3: PN5180DEBUG(F("SET-PASSWORD")); break;
        case 0xBA: PN5180DEBUG(F("ENABLE-PRIVACY")); break;
        default:
          PN5180DEBUG("");
          PN5180DEBUG(formatHex(cmd[b]));
          break;
      }
    } else {
      PN5180DEBUG(formatHex(cmd[b]));
    }
  }
  PN5180DEBUG("]\n");
#endif

  sendData(cmd, cmdLen);
  delay(10);

  uint32_t irqR = getIRQStatus();
  if (0 == (irqR & RX_SOF_DET_IRQ_STAT)) {
	PN5180DEBUG("Didnt detect RX_SOF_DET_IRQ_STAT after sendData");
	return EC_NO_CARD;
  }
  
  // The following line prevents code from continuing until the RX_IRQ_STAT flag 
  // is set in the IRQ_STATUS register, which signifies the end of RF reception
  // But, should the card be removed from the reader quickly before reception is complete,
  // this bit will never be received and the code will hang at this point.
  // TODO implement a reasonable timeout in which to expect the bit to be read, as in https://github.com/playfultechnology/PN5180-Library/blob/e64468d65906c207bdaf900a1645615c71f3d5ca/PN5180.cpp#L489-L494
  /*
  while(!(irqR & RX_IRQ_STAT)) {
	delay(1);
	irqR = getIRQStatus();
  }
  */
  unsigned long startedWaiting = millis();
  while (!(irqR & RX_IRQ_STAT)) {
    irqR = getIRQStatus();
    if (millis() - startedWaiting > commandTimeout) {
      PN5180DEBUG("Didnt detect RX_IRQ_STAT after sendData");
      return EC_NO_CARD;
    }
  }

  uint32_t rxStatus;
  readRegister(RX_STATUS, &rxStatus);
  uint16_t len = (uint16_t)(rxStatus & 0x1ff);

  *resultPtr = readData(len);
  if (0L == *resultPtr) {
    PN5180DEBUG(F("*** ERROR in readData!\n"));
    return ISO15693_EC_UNKNOWN_ERROR;
  }

  uint32_t irqStatus = getIRQStatus();
  if (0 == (RX_SOF_DET_IRQ_STAT & irqStatus)) { // no card detected
     PN5180DEBUG("Didnt detect RX_SOF_DET_IRQ_STAT after readData");
     clearIRQStatus(TX_IRQ_STAT | IDLE_IRQ_STAT);
     return EC_NO_CARD;
  }

  uint8_t responseFlags = (*resultPtr)[0];
  if (responseFlags & (1<<0)) { // error flag
    uint8_t errorCode = (*resultPtr)[1];

    PN5180DEBUG("ERROR code=");
    PN5180DEBUG(formatHex(errorCode));
    PN5180DEBUG(" - ");
    PN5180DEBUG(strerror((ISO15693ErrorCode)errorCode));
    PN5180DEBUG("\n");

    if (errorCode >= 0xA0) { // custom command error codes
      return ISO15693_EC_CUSTOM_CMD_ERROR;
    }
    else return (ISO15693ErrorCode)errorCode;
  }

#ifdef DEBUG
  if (responseFlags & (1<<3)) { // extendsion flag
    PN5180DEBUG("Extension flag is set!\n");
  }
#endif

  clearIRQStatus(RX_SOF_DET_IRQ_STAT | IDLE_IRQ_STAT | TX_IRQ_STAT | RX_IRQ_STAT);
  return ISO15693_EC_OK;
}

bool PN5180ISO15693::setupRF() {
  if (!loadRFConfig(0x0d, 0x8d)) return false;  // ISO15693 parameters
  if (!setRF_on()) return false;

  writeRegisterWithAndMask(SYSTEM_CONFIG, 0xfffffff8);  // Idle/StopCom Command
  writeRegisterWithOrMask(SYSTEM_CONFIG, 0x00000003);   // Transceive Command

  return true;
}

const __FlashStringHelper *PN5180ISO15693::strerror(ISO15693ErrorCode err) {
  PN5180DEBUG(F("ISO15693ErrorCode="));
  PN5180DEBUG(err);
  PN5180DEBUG("\n");
  
  switch (err) {
    case EC_NO_CARD: return F("No card detected!");
    case ISO15693_EC_OK: return F("OK!");
    case ISO15693_EC_NOT_SUPPORTED: return F("Command is not supported!");
    case ISO15693_EC_NOT_RECOGNIZED: return F("Command is not recognized!");
    case ISO15693_EC_OPTION_NOT_SUPPORTED: return F("Option is not supported!");
    case ISO15693_EC_UNKNOWN_ERROR: return F("Unknown error!");
    case ISO15693_EC_BLOCK_NOT_AVAILABLE: return F("Specified block is not available!");
    case ISO15693_EC_BLOCK_ALREADY_LOCKED: return F("Specified block is already locked!");
    case ISO15693_EC_BLOCK_IS_LOCKED: return F("Specified block is locked and cannot be changed!");
    case ISO15693_EC_BLOCK_NOT_PROGRAMMED: return F("Specified block was not successfully programmed!");
    case ISO15693_EC_BLOCK_NOT_LOCKED: return F("Specified block was not successfully locked!");
    default:
      if ((err >= 0xA0) && (err <= 0xDF)) {
        return F("Custom command error code!");
      }
      else return F("Undefined error code in ISO15693!");
  }
}
