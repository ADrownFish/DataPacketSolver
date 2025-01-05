#include "DataPacketSolver.h"

static DataPacketSolver __solver;

uint32_t calculateCRC32(const uint8_t* data, uint32_t len)
{
  // 多项式除数 0xEDB88320
  const uint8_t* bytes = data;
  uint32_t crc = 0xFFFFFFFFUL;
  uint32_t i = 0;
  uint32_t j = 0;

  for (i = 0; i < len; i++) {
    crc ^= bytes[i];
    for (j = 0; j < 8; j++) {
      if (crc & 1) {
        crc = (crc >> 1) ^ 0xEDB88320UL;
      } else {
        crc >>= 1;
      }
    }
  }
  return ~crc;
}

void DPS_setSolver(uint8_t deviceID, uint8_t enableFilter)
{
  __solver._deviceID = deviceID;
  __solver._enableFilter = enableFilter;
}

uint8_t init()
{
  __solver.receivedBufferLength = 0;
  __solver._enableCache = 0;
  __solver._readyRead = 0;
  return 1;
}

void DPS_init() {
  init();
}

uint8_t FrameheaderSniffing(uint8_t byte)
{
  uint8_t _Offset_Uint = 0x01;
  if (byte == HEAD_FLAG3 && __solver._SniffingProgress == _Offset_Uint << 2) {
    __solver._SniffingProgress = _Offset_Uint << 3;
    return 1;
  } else if (byte == HEAD_FLAG2 && __solver._SniffingProgress == _Offset_Uint << 1) {
    __solver._SniffingProgress = _Offset_Uint << 2;
  } else if (byte == HEAD_FLAG1 && __solver._SniffingProgress == _Offset_Uint << 0) {
    __solver._SniffingProgress = _Offset_Uint << 1;
  } else if (byte == HEAD_FLAG0) {
    __solver._SniffingProgress = _Offset_Uint;
  } else {
    __solver._SniffingProgress = 0;
  }
  return 0;
}

uint8_t DPS_pushByte(uint8_t byte) {
  if (FrameheaderSniffing(byte)) {
    init();

    __solver._enableCache = 1;
    __solver._recv_buffer_raw[0] = (HEAD_FLAG0);
    __solver._recv_buffer_raw[1] = (HEAD_FLAG1);
    __solver._recv_buffer_raw[2] = (HEAD_FLAG2);
    __solver._recv_buffer_raw[3] = (HEAD_FLAG3);
    __solver.receivedBufferLength = 4;

    return 0;
  }

  if (__solver._enableCache) {
    __solver._recv_buffer_raw[__solver.receivedBufferLength++] = byte;

    if (__solver.receivedBufferLength > (BUFFER_SIZE + 13)) {
      init();
    } else if (__solver.receivedBufferLength >= 13 &&
               ((__solver._recv_buffer_raw[7]) | __solver._recv_buffer_raw[8]
                                                     << 8) ==
                   (__solver.receivedBufferLength - 13)) {

      uint32_t crc = calculateCRC32(__solver._recv_buffer_raw,
                                    __solver.receivedBufferLength - 4);

      if (((uint8_t *)&crc)[3] == __solver._recv_buffer_raw[(__solver.receivedBufferLength - 1)] &&
          ((uint8_t *)&crc)[2] == __solver._recv_buffer_raw[(__solver.receivedBufferLength - 2)] &&
          ((uint8_t *)&crc)[1] == __solver._recv_buffer_raw[(__solver.receivedBufferLength - 3)] &&
          ((uint8_t *)&crc)[0] == __solver._recv_buffer_raw[(__solver.receivedBufferLength - 4)]) {

        __solver._enableCache = 0;
        __solver._recv_head_raw.srcID = __solver._recv_buffer_raw[4];
        __solver._recv_head_raw.dstID = __solver._recv_buffer_raw[5];
        __solver._recv_head_raw.DataID = __solver._recv_buffer_raw[6];
        __solver._recv_length_raw = ((__solver._recv_buffer_raw[7]) |
                                     __solver._recv_buffer_raw[8] << 8);

        if (__solver._enableFilter) {
          if (__solver._recv_head_raw.dstID == __solver._deviceID) {
            __solver._recv_dataPacket.head.srcID = __solver._recv_head_raw.srcID;
            __solver._recv_dataPacket.head.dstID = __solver._recv_head_raw.dstID;
            __solver._recv_dataPacket.head.DataID = __solver._recv_head_raw.DataID;
            __solver._recv_dataPacket.length = __solver._recv_length_raw;
            memcpy(__solver._recv_dataPacket.data, &__solver._recv_buffer_raw[9], __solver._recv_length_raw);
            __solver._readyRead = 1;
            return 1;
          } else {
            return 0;
          }
        } else {
          __solver._recv_dataPacket.head.srcID = __solver._recv_head_raw.srcID;
          __solver._recv_dataPacket.head.dstID = __solver._recv_head_raw.dstID;
          __solver._recv_dataPacket.head.DataID = __solver._recv_head_raw.DataID;
          __solver._recv_dataPacket.length = __solver._recv_length_raw;
          memcpy(__solver._recv_dataPacket.data, &__solver._recv_buffer_raw[9], __solver._recv_length_raw);
          __solver._readyRead = 1;
        }
      } else {
      }
    }
  }
  return 0;
}

ByteArrayPointer* DPS_makeDataPacket(DPS_Data* data)
{
	static ByteArrayPointer dap;
	uint32_t _crc32 = 0;
	uint16_t totalByteSize = 13 + data->length;

	if (totalByteSize > BUFFER_SIZE + 13)	{
		return &dap;
	}

	__solver._send_buffer_raw[0] = (HEAD_FLAG0);
	__solver._send_buffer_raw[1] = (HEAD_FLAG1);
	__solver._send_buffer_raw[2] = (HEAD_FLAG2);
	__solver._send_buffer_raw[3] = (HEAD_FLAG3);
	__solver._send_buffer_raw[4] = __solver._deviceID;
	__solver._send_buffer_raw[5] = data->head.dstID;
	__solver._send_buffer_raw[6] = (data->head.DataID);
	__solver._send_buffer_raw[7] = (data->length & 0xff);
	__solver._send_buffer_raw[8] = (data->length >> 8);

	uint16_t i = 0;
	for (i = 0; i < data->length; i++)	{
    __solver._send_buffer_raw[9 + i] = (data->data[i]);
	}

	_crc32 = calculateCRC32(__solver._send_buffer_raw, totalByteSize - 4);
	__solver._send_buffer_raw[9 + i + 0] = (((uint8_t*)&_crc32)[0]);
	__solver._send_buffer_raw[9 + i + 1] = (((uint8_t*)&_crc32)[1]);
	__solver._send_buffer_raw[9 + i + 2] = (((uint8_t*)&_crc32)[2]);
	__solver._send_buffer_raw[9 + i + 3] = (((uint8_t*)&_crc32)[3]);

	dap.data = __solver._send_buffer_raw;
	dap.size = totalByteSize;

	return &dap;
}

DPS_Data * DPS_getDataPacket()
{
  if (__solver._readyRead)  {
    __solver._readyRead = 0;
    return &__solver._recv_dataPacket;
  }
  else  {
    return 0;
  }
}

uint8_t DPS_readyRead()
{
  return __solver._readyRead;
}

