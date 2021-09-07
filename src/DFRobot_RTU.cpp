/*!
 * @file DFRobot_RTU.cpp
 * @brief Modbus RTU libary for Arduino. 
 *
 * @copyright   Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @licence     The MIT License (MIT)
 * @author [Arya](xue.peng@dfrobot.com)
 * @version  V1.0
 * @date  2021-07-16
 * @https://github.com/DFRobot/DFRobot_RTU
 */
#include <Arduino.h>
#include "DFRobot_RTU.h"

DFRobot_RTU::DFRobot_RTU(Stream *s)
  : _s(s), _timeout(100){}

DFRobot_RTU::DFRobot_RTU()
  : _s(NULL), _timeout(100){}

void DFRobot_RTU::setTimeoutTimeMs(uint32_t timeout){
  _timeout = timeout;
}

bool DFRobot_RTU::readCoilsRegister(uint8_t id, uint16_t reg){
  uint8_t temp[] = {(uint8_t)((reg >> 8) & 0xFF), (uint8_t)(reg & 0xFF), 0x00, 0x01};
  bool val = false;
  uint8_t ret = 0;
  if((id == 0) && (id > 0xF7)){
      RTU_DBG("Device id error");
      return 0;
  }
  pRtuPacketHeader_t header = packed(id, eCMD_READ_COILS, temp, sizeof(temp));
  sendPackage(header);
  header = recvAndParsePackage(id, (uint8_t)eCMD_READ_COILS, 1, &ret);
  if((ret == 0) && (header != NULL)){
      if (header->payload[1] & 0x01) val = true;
      free(header);
  }
  RTU_DBG(val, HEX);
  return val;
}

bool DFRobot_RTU::readDiscreteInputsRegister(uint8_t id, uint16_t reg){
  uint8_t temp[] = {(uint8_t)((reg >> 8) & 0xFF), (uint8_t)(reg & 0xFF), 0x00, 0x01};
  bool val = false;
  uint8_t ret = 0;
  if((id == 0) && (id > 0xF7)){
      RTU_DBG("Device id error");
      return 0;
  }
  pRtuPacketHeader_t header = packed(id, eCMD_READ_DISCRETE, temp, sizeof(temp));
  sendPackage(header);
  header = recvAndParsePackage(id, (uint8_t)eCMD_READ_DISCRETE, 1, &ret);
  if((ret == 0) && (header != NULL)){
      if (header->payload[1] & 0x01) val = true;
      free(header);
  }
  RTU_DBG(val, HEX);
  return val;
}

uint16_t DFRobot_RTU::readHoldingRegister(uint8_t id, uint16_t reg){
  uint8_t temp[] = {(uint8_t)((reg >> 8) & 0xFF), (uint8_t)(reg & 0xFF), 0x00, 0x01};
  uint16_t val = 0;
  uint8_t ret = 0;
  if((id == 0) && (id > 0xF7)){
      RTU_DBG("Device id error");
      return 0;
  }
  pRtuPacketHeader_t header = packed(id, eCMD_READ_HOLDING, temp, sizeof(temp));
  sendPackage(header);
  header = recvAndParsePackage(id, (uint8_t)eCMD_READ_HOLDING, 2, &ret);
  if((ret == 0) && (header != NULL)){
      val = (header->payload[1] << 8) | header->payload[2];
      free(header);
  }
  //RTU_DBG(val, HEX);
  return val;
}

uint16_t DFRobot_RTU::readInputRegister(uint8_t id, uint16_t reg){
  uint8_t temp[] = {(uint8_t)((reg >> 8) & 0xFF), (uint8_t)(reg & 0xFF), 0x00, 0x01};
  uint16_t val = 0;
  uint8_t ret = 0;
  if((id == 0) && (id > 0xF7)){
      RTU_DBG("Device id error");
      return 0;
  }
  pRtuPacketHeader_t header = packed(id, eCMD_READ_INPUT, temp, sizeof(temp));
  sendPackage(header);
  header = recvAndParsePackage(id, (uint8_t)eCMD_READ_INPUT, 2, &ret);
  if((ret == 0) && (header != NULL)){
      val = (header->payload[1] << 8) | header->payload[2];
      free(header);
  }
  //RTU_DBG(val, HEX);
  return val;
}

uint8_t DFRobot_RTU::writeCoilsRegister(uint8_t id, uint16_t reg, bool flag){
  uint16_t val = flag ? 0xFF00 : 0x0000;
  uint8_t temp[] = {(uint8_t)((reg >> 8) & 0xFF), (uint8_t)(reg & 0xFF), (uint8_t)((val >> 8) & 0xFF), (uint8_t)(val & 0xFF)};
  bool re = !flag;
  uint8_t ret = 0;
  if(id > 0xF7){
      RTU_DBG("Device id error");
      return 0;
  }
  pRtuPacketHeader_t header = packed(id, eCMD_WRITE_COILS, temp, sizeof(temp));
  sendPackage(header);
  header = recvAndParsePackage(id, (uint8_t)eCMD_WRITE_COILS, reg, &ret);
  //if((ret == 0) && (header != NULL)){
  //    if(val == (((header->payload[2] << 8) | header->payload[3]) & 0xFFFF)){
  //      re = flag;
  //    }
  //    free(header);
  //}
  //RTU_DBG(val, HEX);
  return ret;
}
uint8_t DFRobot_RTU::writeHoldingRegister(uint8_t id, uint16_t reg, uint16_t val){
  uint8_t temp[] = {(uint8_t)((reg >> 8) & 0xFF), (uint8_t)(reg & 0xFF), (uint8_t)((val >> 8) & 0xFF), (uint8_t)(val & 0xFF)};
  uint8_t ret = 0;
  if(id > 0xF7){
      RTU_DBG("Device id error");
      return 0;
  }
  pRtuPacketHeader_t header = packed(id, eCMD_WRITE_HOLDING, temp, sizeof(temp));
  sendPackage(header);
  header = recvAndParsePackage(id, (uint8_t)eCMD_WRITE_HOLDING, reg, &ret);
  //val = 0xFFFF;
  //if((ret == 0) && (header != NULL)){
  //    val = (header->payload[2] << 8) | header->payload[3];
  //    free(header);
  //}
  //RTU_DBG(val, HEX);
  return ret;
}

uint8_t DFRobot_RTU::readCoilsRegister(uint8_t id, uint16_t reg, uint16_t regNum, uint8_t *data, uint16_t size){
  uint8_t length = regNum/8 + ((regNum%8) ? 1 : 0);
  uint8_t temp[] = {(uint8_t)((reg >> 8) & 0xFF), (uint8_t)(reg & 0xFF), (uint8_t)((regNum >> 8) & 0xFF), (uint8_t)(regNum & 0xFF)};
  uint16_t val = 0;
  uint8_t ret = 0;
  if((id == 0) && (id > 0xF7)){
      RTU_DBG("Device id error");
      return eRTU_ID_ERROR;
  }
  pRtuPacketHeader_t header = packed(id, eCMD_READ_COILS, temp, sizeof(temp));
  sendPackage(header);
  header = recvAndParsePackage(id, (uint8_t)eCMD_READ_COILS, length, &ret);
  if((ret == 0) && (header != NULL)){
      if(data != NULL){
         size = (size > length) ? length : size;
         memcpy(data, (uint8_t *)&(header->payload[1]), size);
      } 
      free(header);
  }
  RTU_DBG(val, HEX);
  return ret;
}
uint8_t DFRobot_RTU::readDiscreteInputsRegister(uint8_t id, uint16_t reg, uint16_t regNum, uint8_t *data, uint16_t size){
  uint8_t length = regNum/8 + ((regNum%8) ? 1 : 0);
  uint8_t temp[] = {(uint8_t)((reg >> 8) & 0xFF), (uint8_t)(reg & 0xFF), (uint8_t)((regNum >> 8) & 0xFF), (uint8_t)(regNum & 0xFF)};
  uint16_t val = 0;
  uint8_t ret = 0;
  if((id == 0) && (id > 0xF7)){
      RTU_DBG("Device id error");
      return eRTU_ID_ERROR;
  }
  pRtuPacketHeader_t header = packed(id, eCMD_READ_DISCRETE, temp, sizeof(temp));
  sendPackage(header);
  header = recvAndParsePackage(id, (uint8_t)eCMD_READ_DISCRETE, length, &ret);
  if((ret == 0) && (header != NULL)){
      if(data != NULL){
         size = (size > length) ? length : size;
         memcpy(data, (uint8_t *)&(header->payload[1]), size);
      } 
      free(header);
  }
  RTU_DBG(val, HEX);
  return ret;
}
uint8_t DFRobot_RTU::readHoldingRegister(uint8_t id, uint16_t reg, void *data, uint16_t size){
  uint8_t length = size/2 + ((size%2) ? 1 : 0);
  uint8_t temp[] = {(uint8_t)((reg >> 8) & 0xFF), (uint8_t)(reg & 0xFF), (uint8_t)((length >> 8) & 0xFF), (uint8_t)(length & 0xFF)};
  uint16_t val = 0;
  uint8_t ret = 0;
  if((id == 0) && (id > 0xF7)){
      RTU_DBG("Device id error");
      return eRTU_ID_ERROR;
  }
  pRtuPacketHeader_t header = packed(id, eCMD_READ_HOLDING, temp, sizeof(temp));
  sendPackage(header);
  header = recvAndParsePackage(id, (uint8_t)eCMD_READ_HOLDING, length*2, &ret);
  if((ret == 0) && (header != NULL)){
      if(data != NULL) memcpy(data, (uint8_t *)&(header->payload[1]), size);
      free(header);
  }
  RTU_DBG(val, HEX);
  return ret;
}

uint8_t DFRobot_RTU::readInputRegister(uint8_t id, uint16_t reg, void *data, uint16_t size){
  uint8_t length = size/2 + ((size%2) ? 1 : 0);
  uint8_t temp[] = {(uint8_t)((reg >> 8) & 0xFF), (uint8_t)(reg & 0xFF), (uint8_t)((length >> 8) & 0xFF), (uint8_t)(length & 0xFF)};
  uint16_t val = 0;
  uint8_t ret = 0;
  if((id == 0) && (id > 0xF7)){
      RTU_DBG("Device id error");
      return eRTU_ID_ERROR;
  }
  pRtuPacketHeader_t header = packed(id, eCMD_READ_INPUT, temp, sizeof(temp));
  sendPackage(header);
  header = recvAndParsePackage(id, (uint8_t)eCMD_READ_INPUT, length*2, &ret);
  if((ret == 0) && (header != NULL)){
      if(data != NULL) memcpy(data, (uint8_t *)&(header->payload[1]), size);
      free(header);
  }
  RTU_DBG(val, HEX);
  return ret;
}

uint8_t DFRobot_RTU::readHoldingRegister(uint8_t id, uint16_t reg, uint16_t *data, uint16_t regNum){
  uint8_t temp[] = {(uint8_t)((reg >> 8) & 0xFF), (uint8_t)(reg & 0xFF), (uint8_t)((regNum >> 8) & 0xFF), (uint8_t)(regNum & 0xFF)};
  uint16_t val = 0;
  uint8_t ret = 0;
  if((id == 0) && (id > 0xF7)){
      RTU_DBG("Device id error");
      return eRTU_ID_ERROR;
  }
  pRtuPacketHeader_t header = packed(id, eCMD_READ_HOLDING, temp, sizeof(temp));
  sendPackage(header);
  header = recvAndParsePackage(id, (uint8_t)eCMD_READ_HOLDING, regNum*2, &ret);
  if((ret == 0) && (header != NULL)){
      if(data != NULL){
        for(int i = 0; i < regNum; i++){
          data[i] = ((header->payload[1+2*i]) << 8) | (header->payload[2+2*i]);
        }
      } 
      free(header);
  }
  return ret;
}

uint8_t DFRobot_RTU::readInputRegister(uint8_t id, uint16_t reg, uint16_t *data, uint16_t regNum){
  uint8_t temp[] = {(uint8_t)((reg >> 8) & 0xFF), (uint8_t)(reg & 0xFF), (uint8_t)((regNum >> 8) & 0xFF), (uint8_t)(regNum & 0xFF)};
  uint16_t val = 0;
  uint8_t ret = 0;
  if((id == 0) && (id > 0xF7)){
      RTU_DBG("Device id error");
      return eRTU_ID_ERROR;
  }
  pRtuPacketHeader_t header = packed(id, eCMD_READ_INPUT, temp, sizeof(temp));
  sendPackage(header);
  header = recvAndParsePackage(id, (uint8_t)eCMD_READ_INPUT, regNum*2, &ret);
  if((ret == 0) && (header != NULL)){
      if(data != NULL){
        for(int i = 0; i < regNum; i++){
          data[i] = ((header->payload[1+2*i]) << 8) | (header->payload[2+2*i]);
        }
      } 
      free(header);
  }
  return ret;
}

uint8_t DFRobot_RTU::writeCoilsRegister(uint8_t id, uint16_t reg, uint16_t regNum, uint8_t *data, uint16_t size){
  uint16_t length = regNum/8 + ((regNum%8) ? 1 : 0);
  if(size < length) return (uint8_t)eRTU_EXCEPTION_ILLEGAL_DATA_VALUE;
  #if defined(ESP8266)
  uint8_t temp[size + 5];
  temp[0] = (uint8_t)((reg >> 8) & 0xFF);
  temp[1] = (uint8_t)(reg & 0xFF);
  temp[2] = (uint8_t)((regNum >> 8) & 0xFF);
  temp[3] = (uint8_t)(regNum & 0xFF);
  temp[4] = (uint8_t)length;
  #else
  uint8_t temp[size + 5] = {(uint8_t)((reg >> 8) & 0xFF), (uint8_t)(reg & 0xFF),(uint8_t)((regNum >> 8) & 0xFF), (uint8_t)(regNum & 0xFF),(uint8_t)length};
  #endif
  uint8_t ret = 0;
  if(id > 0xF7){
      RTU_DBG("Device id error");
      return (uint8_t)eRTU_ID_ERROR;
  }
  memcpy(temp+5, data, size);
  pRtuPacketHeader_t header = packed(id, eCMD_WRITE_MULTI_COILS, temp, sizeof(temp));
  sendPackage(header);
  header = recvAndParsePackage(id, (uint8_t)eCMD_WRITE_MULTI_COILS, reg, &ret);
  size = 0;
  if((ret == 0) && (header != NULL)){
      size = (header->payload[2] << 8) | header->payload[3];
      free(header);
  }
  return ret;
}
uint8_t DFRobot_RTU::writeHoldingRegister(uint8_t id, uint16_t reg, void *data, uint16_t size){
  if(((size % 2) != 0) || (size > 250) || data == NULL) return (uint8_t)eRTU_EXCEPTION_ILLEGAL_DATA_VALUE;
  #if defined(ESP8266)
  uint8_t temp[size + 5];
  temp[0] = (uint8_t)((reg >> 8) & 0xFF);
  temp[1] = (uint8_t)(reg & 0xFF);
  temp[2] = (uint8_t)(((size/2) >> 8) & 0xFF);
  temp[3] = (uint8_t)((size/2) & 0xFF);
  temp[4] = (uint8_t)size;
  #else
  uint8_t temp[size + 5] = {(uint8_t)((reg >> 8) & 0xFF), (uint8_t)(reg & 0xFF), (uint8_t)(((size/2) >> 8) & 0xFF), (uint8_t)((size/2) & 0xFF),(uint8_t)size};
  #endif
  uint8_t ret = 0;
  if(id > 0xF7){
      RTU_DBG("Device id error");
      return (uint8_t)eRTU_ID_ERROR;
  }
  memcpy(temp+5, data, size);
  pRtuPacketHeader_t header = packed(id, eCMD_WRITE_MULTI_HOLDING, temp, sizeof(temp));
  sendPackage(header);
  header = recvAndParsePackage(id, (uint8_t)eCMD_WRITE_MULTI_HOLDING, reg, &ret);
  size = 0;
  if((ret == 0) && (header != NULL)){
      size = (header->payload[2] << 8) | header->payload[3];
      free(header);
  }
  return ret;
}

uint8_t DFRobot_RTU::writeHoldingRegister(uint8_t id, uint16_t reg, uint16_t *data, uint16_t regNum){
  uint16_t size = regNum * 2;
  uint8_t *pBuf = (uint8_t *)data;
  #if defined(ESP8266)
  uint8_t temp[size + 5];
  temp[0] = (uint8_t)((reg >> 8) & 0xFF);
  temp[1] = (uint8_t)(reg & 0xFF);
  temp[2] = (uint8_t)(((size/2) >> 8) & 0xFF);
  temp[3] = (uint8_t)((size/2) & 0xFF);
  temp[4] = (uint8_t)size;
  #else
  uint8_t temp[size + 5] = {(uint8_t)((reg >> 8) & 0xFF), (uint8_t)(reg & 0xFF), (uint8_t)(((size/2) >> 8) & 0xFF), (uint8_t)((size/2) & 0xFF),(uint8_t)size};
  #endif
  uint8_t ret = 0;
  if(id > 0xF7){
      RTU_DBG("Device id error");
      return (uint8_t)eRTU_ID_ERROR;
  }
  //memcpy(temp+5, data, size);
  for(int i = 0; i < regNum; i++){
     temp[5+i] =  pBuf[2*i + 1];
     temp[6+i] =  pBuf[2*i] ;
  }
  pRtuPacketHeader_t header = packed(id, eCMD_WRITE_MULTI_HOLDING, temp, sizeof(temp));
  sendPackage(header);
  header = recvAndParsePackage(id, (uint8_t)eCMD_WRITE_MULTI_HOLDING, reg, &ret);
  size = 0;
  if((ret == 0) && (header != NULL)){
      size = (header->payload[2] << 8) | header->payload[3];
      free(header);
  }
  return ret;
}

DFRobot_RTU::pRtuPacketHeader_t DFRobot_RTU::packed(uint8_t id, eFunctionCommand_t cmd, void *data, uint16_t size){
  return packed(id, (uint8_t)cmd, data, size);
}

DFRobot_RTU::pRtuPacketHeader_t DFRobot_RTU::packed(uint8_t id, uint8_t cmd, void *data, uint16_t size){
  pRtuPacketHeader_t header = NULL;
  uint16_t crc = 0;
  if((data == NULL) || (size == 0)) return NULL;
  if((header = (pRtuPacketHeader_t)malloc(sizeof(sRtuPacketHeader_t) + size)) == NULL){
      RTU_DBG("Memory ERROR");
      return NULL;
  }
  header->len = sizeof(sRtuPacketHeader_t) + size - 2;
  header->id = id;
  header->cmd = cmd;
  memcpy(header->payload, data, size);
  crc = calculateCRC((uint8_t *)&(header->id), (header->len) - 2);
  header->payload[size] = (crc >> 8) & 0xFF;
  header->payload[size + 1] = crc & 0xFF;
  return header;
}

void DFRobot_RTU::sendPackage(pRtuPacketHeader_t header){
  clearRecvBuffer();
  if(header != NULL){
      _s->write((uint8_t *)&(header->id), header->len);
      delay(5);
      if(header->id == 0){
        delay(_timeout);
      }
      free(header);
  }
}

DFRobot_RTU::pRtuPacketHeader_t DFRobot_RTU::recvAndParsePackage(uint8_t id, uint8_t cmd, uint16_t data, uint8_t *error){
  if(id > 0xF7) return NULL;
  if(id == 0){//广播包不会有接收数据
    if (error != NULL) *error = 0;
    return NULL;
  }
  
  uint8_t head[4] = {0, 0, 0, 0};
  uint16_t crc = 0;
  uint8_t ret = 0;
  pRtuPacketHeader_t header = NULL;
  uint16_t waitFortimeoutMs = 50;
  uint8_t timeInterMs = 5;
  uint8_t count;

LOOP:
  uint16_t remain, index = 0;
  uint32_t time = millis();
  for(int i = 0; i < 4;){
      if(_s->available()){

          head[index++] = (uint8_t)_s->read();
          RTU_DBG(head[index-1],HEX);
          if((index == 1) && (head[0] != id)){
              index = 0;
          }else if((index == 2) && ((head[1]&0x7F) != cmd)){
              index = 0;
          }
          i = index;
          time = millis();
      }
      if((millis() - time) > _timeout) {
          RTU_DBG("ERROR");
          RTU_DBG(millis() - time);
          break;
      }
  }

  if(index != 4) {
      RTU_DBG();
      if(error != NULL) *error = eRTU_RECV_ERROR;
      return NULL;
  }
  switch(head[1]){
      case eCMD_READ_COILS:
      case eCMD_READ_DISCRETE:
      case eCMD_READ_HOLDING:
      case eCMD_READ_INPUT:
           if(head[2] != (data & 0xFF)) {
               index = 0;
               goto LOOP;
           }
           index = 5 + head[2];
           break;
      case eCMD_WRITE_COILS:
      case eCMD_WRITE_HOLDING:
      case eCMD_WRITE_MULTI_COILS:
      case eCMD_WRITE_MULTI_HOLDING:
           if(((head[2] << 8) | (head[3])) != data){
               index = 0;
               goto LOOP;
           }
           index = 8;
           break;
      default:
           index = 5;
           break;
  }
  if((header = (pRtuPacketHeader_t)malloc(index+2)) == NULL){
      RTU_DBG("Memory ERROR");
      if(error != NULL) *error = eRTU_RECV_ERROR;
      return NULL;
  }
  header->len = index;
  memcpy((uint8_t *)&(header->id), head, 4);
  remain = index - 4;
  index = 2;
  time = millis();
  while(remain){
      RTU_DBG(_s->available());
      if(_s->available()){
          header->payload[index++] = (uint8_t)_s->read();
          time = millis();
          remain--;
      }
      if((millis() - time) > _timeout) {
          free(header);
          if(error != NULL) *error = eRTU_RECV_ERROR;
          RTU_DBG();
          return NULL;
      }
  }
  crc = (header->payload[(header->len)-4] << 8) | header->payload[(header->len)-3];

  if(crc != calculateCRC((uint8_t *)&(header->id), (header->len) - 2)){
      free(header);
      RTU_DBG("CRC ERROR");
      if(error != NULL) *error = eRTU_RECV_ERROR;
      return NULL;
  }
  if(error != NULL) *error = 0;
  if(head[1] & 0x80){
      *error = head[2];
  }
  
  return header;

}


uint16_t DFRobot_RTU::calculateCRC(uint8_t *data, uint8_t len){
  uint16_t crc = 0xFFFF;
  for( uint8_t pos = 0; pos < len; pos++){
    crc ^= (uint16_t)data[ pos ];
    for(uint8_t i = 8; i != 0; i--){
      if((crc & 0x0001) != 0){
        crc >>= 1;
        crc ^= 0xA001;
      }else{
         crc >>= 1;
      }
    }
  }
  crc = ((crc & 0x00FF) << 8) | ((crc & 0xFF00) >> 8);
  return crc;
}

void DFRobot_RTU::clearRecvBuffer(){
  while(_s->available()){
    _s->read();
    delay(2);
  }
}
