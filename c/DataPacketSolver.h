/******************************************************************************
Copyright (c) 2024, ADrownFish. 
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#ifndef DATAPACKETSOLVER_H__
#define DATAPACKETSOLVER_H__

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Robot 数据包 [封包] [解包] 器   【C版本】
 * CRC32校验， 对应【DataPacketSolver】C++ 的CRC32
 * 构造数据包，内容包括帧头、信息、长度、数据、校验等功能
 *
 * @author ADrownFish
 * @author 852976773qq.com
 * @date 2024-04-25
 */

#include <string.h>
#include <stdint.h>

// 单个数据包的大小，如果嵌入式设备没那么多资源，应将此Buffer值设置小一些
#define BUFFER_SIZE 32

//帧头
#define HEAD_FLAG0 0xAA
#define HEAD_FLAG1 0xBB
#define HEAD_FLAG2 0xCC
#define HEAD_FLAG3 0xDD

typedef struct {
  uint32_t size;          // 数组大小
  unsigned char* data;    // 数组指针
} ByteArrayPointer;

typedef struct {
  uint8_t srcID;      // 发送者ID
  uint8_t dstID;      // 目标者ID
  uint8_t DataID;     // 标识数据包的ID
} Head;

typedef struct {
  Head head;
  uint16_t length;
  uint8_t data[BUFFER_SIZE];
} DPS_Data;

typedef struct {
  uint8_t _enableCache;
  uint8_t _readyRead;
  uint8_t _enableFilter;
  uint8_t _deviceID;
  uint8_t _SniffingProgress;
  uint16_t receivedBufferLength;

  Head _recv_head_raw;
  uint16_t _recv_length_raw;
  uint8_t _recv_buffer_raw[BUFFER_SIZE + 13];  
  uint8_t _send_buffer_raw[BUFFER_SIZE + 13]; 
  DPS_Data _recv_dataPacket;
} DataPacketSolver;

//配置本机ID、是否开启过滤器
extern void DPS_setSolver(uint8_t deviceID, uint8_t enableFilter);
//解析单字节
extern uint8_t DPS_pushByte(uint8_t byte);
//打包
extern ByteArrayPointer* DPS_makeDataPacket(DPS_Data* data);
//获取解析结果
extern DPS_Data* DPS_getDataPacket();
//是否可读
extern uint8_t DPS_readyRead();

#ifdef __cplusplus
}
#endif

#endif


