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

#pragma once

/**
 * @class DataPacketSolver
 * @brief 封包与解包模块，提供数据包的构造与解析功能
 * @brief Packet encapsulation and parsing module, provides data packet construction and analysis functionalities.
 * 
 * @author ADrownFish
 * @email a.drownfishqq.com
 * @date 2023-07-25
 *
 * @version 1.1
 */


#include <string.h>
#include <stdint.h>
#include <cstdlib>
#include <cassert>

#ifdef DATAPACKETSOLVER_PRINT_OPTIONS
#include <iostream>
#include <iomanip>
#define DATAPACKETSOLVER_MSG_RESET "\033[0m"
#define DATAPACKETSOLVER_MSG_RED "\033[31m"   // Red
#define DATAPACKETSOLVER_MSG_GREEN "\033[32m" // Green
#endif

namespace robot
{
  class CRC8;
  class CRC16;
  class CRC24;
  class CRC16_CCITT;
  class CRC32;

  /*
  * @class DataPacketSolver
  * @template <BufferSize, MaxReceiveQueueSize, CRCFunction>
  * 
  * 数据包解析器：负责处理经过校验的可靠数据包，可用于具有指定接收队列长度的缓存系统。
  * The DataPacketSolver is responsible for processing reliable data packets with integrity checks and supports 
  * a configurable queue for buffered data packets.
  * 
  * @param BufferSize 单个数据包的缓冲区大小（字节数），用于指定解析器可处理的最大数据包长度。
  *        The buffer size of a single data packet (in bytes), specifying the maximum length of packets that can be processed.
  * 
  * @param MaxReceiveQueueSize 最大接收队列长度，定义解析器能够缓存的有效数据包数量。
  *        The maximum size of the receive queue, defining the number of valid packets that can be stored in the buffer.
  * 
  * @param CRCFunction 数据包校验所使用的 CRC 方法（默认为 CRC32），用于确保数据完整性与可靠性。
  *        The CRC method used for packet integrity checks (default: CRC32), ensuring data reliability and correctness.
  * 
  */

  template <int BufferSize, int MaxReceiveQueueSize, typename CRCFunction = CRC32>
  class DataPacketSolver{
    static_assert(BufferSize >= 1 && BufferSize <= 65000, "[DataPacketSolver] BufferSize must be between 1 and 65000");
    static_assert(MaxReceiveQueueSize >= 1 && MaxReceiveQueueSize <= 65000, "[DataPacketSolver] MaxReceiveQueueSize must be between 1 and 65000");

  public:

    struct ByteArrayPointer
    {
      uint32_t size = 0x00;    // 数组大小
      uint8_t *data = nullptr; // 数组指针
    };
    
    class Head{
    public:
      Head() = default;
      uint8_t srcID = 0x00;  // 发送者ID
      uint8_t dstID = 0x00;  // 目标者ID
      uint8_t DataID = 0x00; // 标识数据包的ID
    };
    
    class Data{
    public:
      Head head;
      uint16_t length = 0x00;
      uint8_t data[BufferSize] = {};

    public:
      void clear(){
        length = 0;
      }
      /**
       * @brief 设置数据包头部信息
       *
       * @param _dstID 目标者ID
       * @param _dataID 数据包ID
       */
      void setHead(uint8_t _dstID, uint8_t _dataID){
        head.dstID = _dstID;
        head.DataID = _dataID;
      }
      void setHead(uint8_t _srcID ,uint8_t _dstID, uint8_t _dataID){
        head.srcID = _srcID;
        head.dstID = _dstID;
        head.DataID = _dataID;
      }

      /**
       * @brief 设置数据包头部信息
       *
       * @param _head 待设置的头部信息
       */
      void setHead(Head _head){
        head.srcID = _head.srcID;
        head.dstID = _head.dstID;
        head.DataID = _head.DataID;
      }

      /**
       * @brief 设置数据内容
       *
       * @param _data 指向数据的指针
       * @param _dataLength 数据的长度 
       */
      void appendData(const void *_data, uint16_t _dataLength){
        assert(_dataLength != 0);

        if (length + _dataLength > BufferSize){
#ifdef DATAPACKETSOLVER_PRINT_OPTIONS
          std::cout << "[ Solver ][ Data ] Buffer overflow: The array size exceeds the buffer size. Please resize the array within the buffer limits.\n";
#endif
          _dataLength = BufferSize - length; // 限制追加的数据长度，使得总长度不超过缓冲区大小
        }

        memcpy(this->data + length, _data, _dataLength); // 将数据追加到缓冲区的末尾
        length += _dataLength;                           // 更新数据长度
      }

      /**
       * @brief 设置数据内容
       *
       * @tparam T 数据类型
       * @param _data 指向数据的指针
       * @param _size 数据个数
       */
      template <typename T>
      void appendData(const T *_data, uint16_t _size){
        assert(_size != 0);

        uint16_t __size = sizeof(T) * _size;
        const void *__data = _data;
        appendData(__data, __size);
      }

      template <typename T>
      void appendData(const T &_data){
        uint16_t __size = sizeof(T);
        const void *__data = &_data;
        appendData(__data, __size);
      }

      /**
       * @brief 设置数据内容
       *
       * @tparam T 数据类型
       * @tparam Args 可变模板参数类型
       * @param args 数据内容
       *
       * 示例:
       * \code{.cpp}
       * setData<uint8_t>(1, 2, 3, 4, 5);
       * \endcode
       */
      template <typename T, typename... Args>
      void appendData(Args... args){  
        T _data[sizeof...(args)] = {static_cast<T>(args)...}; // 将参数包展开到数组中
        uint16_t _size = sizeof(_data);                       // 计算数据大小
        const void *__data = _data;
        appendData(__data, _size);
      }

      Data(uint8_t srcID,uint8_t dstID,uint8_t dataID,const void * data,uint16_t length){
        appendData(srcID,dstID,dataID,data,length);
      }

      // 深拷贝构造函数
      Data(const Data &other){
        head.srcID  = other.head.srcID;
        head.dstID  = other.head.dstID;
        head.DataID = other.head.DataID;
        length  = other.length;

        // 深拷贝数组内容
        for (uint16_t i = 0; i < length; ++i){
          data[i] = other.data[i];
        }
      }
      Data(){
      }
      Data(const Head & _head,const uint8_t *_data,uint16_t length){
        setHead(_head);
        appendData(_data,length);
      }

      // = 操作符重载
      Data &operator=(const Data &other){
        head.srcID  = other.head.srcID;
        head.dstID  = other.head.dstID;
        head.DataID = other.head.DataID;
        length  = other.length;
        for (uint16_t i = 0; i < length; ++i){
          data[i] = other.data[i];
        }        
        return *this;
      }
    };
    
    class DataQueue{
    private:
      uint16_t startPos = 0;
      uint16_t endPos = 0;
      uint16_t buffer_size = 0;
      Data _recv_buffer_array[MaxReceiveQueueSize];

    public:
      size_t size() const{
        return buffer_size;
      }

      void push(const Data &dps){
        if (buffer_size < MaxReceiveQueueSize)
        {
          buffer_size++;
        }
        else
        {
          // Queue is full, discard the oldest element
          startPos = (startPos + 1) % MaxReceiveQueueSize;
        }

        _recv_buffer_array[endPos] = dps;
        endPos = (endPos + 1) % MaxReceiveQueueSize;
      }

      Data &pop(){
        if (buffer_size == 0){
          static Data data{};
          return data;
        }

        buffer_size--;
        uint16_t startPos_backup = startPos;
        startPos = (startPos + 1) % MaxReceiveQueueSize;
        return _recv_buffer_array[startPos_backup];
      }
    };

  private:

    static constexpr uint8_t _HEAD_FLAG[] = {0xAA,0xBB,0xCC,0xDD};
    static constexpr uint8_t _HEAD_FLAG_LENGTH = sizeof(_HEAD_FLAG);
    static constexpr uint8_t _Head_LENGTH = sizeof(Head);
    static constexpr uint8_t _DATA_LENGTH = sizeof(uint16_t);
    static constexpr uint8_t _FIXED_LENGTH = _HEAD_FLAG_LENGTH + _Head_LENGTH + _DATA_LENGTH;  // fram 4 head 3 length 2 = 9
    static constexpr uint8_t _MAX_CRC_LENGTH = CRCFunction::_CRC_LENGTH;
    static constexpr uint8_t _MAX_PADDING_LENGTH = _FIXED_LENGTH + _MAX_CRC_LENGTH;
    static constexpr uint8_t _MAX_FIXED_LENGTH = _MAX_PADDING_LENGTH + BufferSize;

    bool _enableCache = false;
    bool _enableFilter = true;

#ifdef DATAPACKETSOLVER_PRINT_OPTIONS
    bool _enablePrintRecv = false;
    bool _enablePrintSend = false;
#endif

    uint8_t _deviceID = 0x00;
    uint8_t _SniffingProgress = 0;
    uint16_t receivedBufferLength = 0;

    uint32_t _unpackInvalidCount = 0;
    uint32_t _unpackValidCount = 0;
    
    Head _recv_head_raw;
    uint16_t _recv_length_raw = 0;
    
    uint8_t _recv_buffer_raw[_MAX_FIXED_LENGTH] = {};
    uint8_t _send_buffer_raw[_MAX_FIXED_LENGTH] = {};
    DataQueue _recv_dataPacket_queue;

    CRCFunction crc_;

  public:
    /**
     * @brief 构造函数
     * @param thisID 指定该设备的ID,用于接收数据时,过滤不属于本设备的数据包
     * @param enableFilter 是否启用过滤
     * @return 无返回
     */
    DataPacketSolver(uint8_t deviceID, bool enableFilter = true)    {
      _deviceID = deviceID;
      _enableFilter = enableFilter;

      init();
    }
    DataPacketSolver(){
      init();
    }
    ~DataPacketSolver(){
      
    }

    /**
     * @brief 压入一个字节，尝试解析
     * @param byte 用于解包的字节
     * @return 解包成功 true  失败 false
     */
    bool pushByte(uint8_t byte){
#ifdef DATAPACKETSOLVER_PRINT_OPTIONS
      auto _print__ = [this](bool ok)
      {
        if (_enablePrintRecv)
        {
          std::cout << "[ Solver ][ unpack ]";
          if (ok)          {
            std::cout << DATAPACKETSOLVER_MSG_GREEN "[ Valid ]" DATAPACKETSOLVER_MSG_RESET;
          }
          else          {
            std::cout << DATAPACKETSOLVER_MSG_RED "[ Invalid ]" DATAPACKETSOLVER_MSG_RESET;
          }
          for (size_t i = 0; i < _recv_length_raw; i++)          {
            std::cout << " " << std::hex << std::setfill('0') << std::setw(2) << (int)_recv_buffer_raw[i+9];
          }
          std::cout << std::dec << std::endl;
        }
      };
#endif

      if (FrameheaderSniffing(byte)){
        init();

        _enableCache = true;        
        receivedBufferLength = _HEAD_FLAG_LENGTH;
        _recv_buffer_raw[0] = _HEAD_FLAG[0];
        _recv_buffer_raw[1] = _HEAD_FLAG[1];
        _recv_buffer_raw[2] = _HEAD_FLAG[2];
        _recv_buffer_raw[3] = _HEAD_FLAG[3];
        
        return false;
      }

      if (_enableCache){
        _recv_buffer_raw[receivedBufferLength++] = byte;

        if(receivedBufferLength > (_MAX_FIXED_LENGTH)){
          init();          
        }else if (receivedBufferLength >= _MAX_PADDING_LENGTH &&
                  (_recv_buffer_raw[7] + (_recv_buffer_raw[8] << 8)) == (receivedBufferLength - _MAX_PADDING_LENGTH)){

          uint32_t _crcRes = crc_.crc(_recv_buffer_raw, receivedBufferLength - _MAX_CRC_LENGTH);
          const uint8_t* pCRC = ((uint8_t *)&_crcRes);
          const uint8_t* pData = &_recv_buffer_raw[(receivedBufferLength - _MAX_CRC_LENGTH)];

          uint8_t times = 0;
          for (uint8_t i = 0; i < _MAX_CRC_LENGTH; i++){
            if(pData[i] == pCRC[i]){
              times++;
            }
          }

          if(times == _MAX_CRC_LENGTH){
            _enableCache = false;
            _unpackValidCount++;

            _recv_head_raw.srcID = _recv_buffer_raw[4];
            _recv_head_raw.dstID = _recv_buffer_raw[5];
            _recv_head_raw.DataID = _recv_buffer_raw[6];
            _recv_length_raw = ((_recv_buffer_raw[7]) | _recv_buffer_raw[8] << 8);

#ifdef DATAPACKETSOLVER_PRINT_OPTIONS
            _print__(true);
#endif

            // 是否开启了过滤
            if (_enableFilter){
              if (_recv_head_raw.dstID == _deviceID){
                _recv_dataPacket_queue.push(Data(_recv_head_raw,&_recv_buffer_raw[9],_recv_length_raw));
                return true;
              }
              else{
                return false;
              }
            }
            else{
              _recv_dataPacket_queue.push(Data(_recv_head_raw,&_recv_buffer_raw[9],_recv_length_raw));
            }
          }
          else{
            _unpackInvalidCount++;

#ifdef DATAPACKETSOLVER_PRINT_OPTIONS            
            _print__(false);
#endif            
          }
        }
      }
      return false;
    }

    /**
     * @brief 压入一堆字节，解析出来的会放到队列里面
     * @param data 用于解包的字节地址
     * @param length 用于解包的字节长度
     * @return 无返回
     */
    void pushBytes(const uint8_t *data, uint16_t length){
      for (uint16_t i = 0; i < length; i++){
        pushByte(data[i]);
      }
      return;
    }

    /**
     * @brief 是否可用包
     * @return true false
     */
    int getAvailableSize() const{
      return _recv_dataPacket_queue.size();
    }

    /**
     * @brief 获取一个包
     * @return 有可用返回结构体 无可用返回空结构
     */
    const Data &getFirstDataPacket(){
      return _recv_dataPacket_queue.pop();
    }    

    /**
     * @brief 构造结构体，添加帧头校验等
     * @param data 用于构造的数据结构
     * @return 构造完成的数据流
     */
    const ByteArrayPointer makeDataPacket(const Data &data){
      ByteArrayPointer dpba;
      const uint16_t totalByteSize = _FIXED_LENGTH + data.length + _MAX_CRC_LENGTH;

      // 如果超出缓冲区，则返回空
      if (totalByteSize > _MAX_FIXED_LENGTH){
#ifdef DATAPACKETSOLVER_PRINT_OPTIONS
        std::cout << "[ Solver ][ makeDataPacket ] Buffer overflow: The array size exceeds the buffer size. Please resize the array within the buffer limits.\n";
#endif
        return dpba;
      }

      _send_buffer_raw[0] = (_HEAD_FLAG[0]);
      _send_buffer_raw[1] = (_HEAD_FLAG[1]);
      _send_buffer_raw[2] = (_HEAD_FLAG[2]);
      _send_buffer_raw[3] = (_HEAD_FLAG[3]);
      _send_buffer_raw[4] = this->_deviceID;
      _send_buffer_raw[5] = data.head.dstID;
      _send_buffer_raw[6] = (data.head.DataID);
      _send_buffer_raw[7] = (data.length & 0xff);
      _send_buffer_raw[8] = (data.length >> 8);

      uint16_t i = 0;
      for (i = 0; i < data.length; i++){
        _send_buffer_raw[_FIXED_LENGTH + i] = (data.data[i]);
      }

      uint32_t _crcRes = crc_.crc(_send_buffer_raw, totalByteSize - _MAX_CRC_LENGTH);
      const uint8_t* pCRC = ((uint8_t *)&_crcRes);      

      for (i = 0; i < _MAX_CRC_LENGTH; i++){
        _send_buffer_raw[_FIXED_LENGTH + data.length + i] = pCRC[i];
      }

#ifdef DATAPACKETSOLVER_PRINT_OPTIONS
      if (_enablePrintSend){
        std::cout << "[ Solver ][ pack ]";

        for (size_t i = 0; i < totalByteSize; i++){
          std::cout << " " << std::hex << std::setfill('0') << std::setw(2) << (int)_send_buffer_raw[i];
        }

        std::cout << std::dec << std::endl;
      }
#endif      

      dpba.data = _send_buffer_raw;
      dpba.size = totalByteSize;

      return dpba;
    }

    /**
     * @brief 设置Device ID
     * @param deviceID ID值
     * @return noreturn
     */
    inline void setDeviceID(uint8_t deviceID){
      _deviceID = deviceID;
    }
    inline uint8_t getDeviceID() const{
      return _deviceID;
    }
    /**
     * @brief 设置启用滤波器
     * @param enableFilter 启用
     * @return noreturn
     */
    inline void setEnableFilter(bool enableFilter){
      _enableFilter = enableFilter;
    }

#ifdef DATAPACKETSOLVER_PRINT_OPTIONS
    /**
     * @brief 设置开启接收数据打印
     * @param ok 是否
     * @return noreturn
     */
    void setEnablePrintReceive(bool ok){
      _enablePrintRecv = ok;
    }

    /**
     * @brief 设置开启发送数据打印
     * @param ok 是否
     * @return noreturn
     */
    void setEnablePrintSend(bool ok){
      _enablePrintSend = ok;
    }

#endif

    inline uint32_t getUnpackInvalidCount() const {
      return _unpackInvalidCount;
    }

    inline uint32_t getUnpackValidCount() const {
      return _unpackValidCount;
    }

  private:
    /**
     * @brief 帧头识别，内部用
     * @param byte 用于判定帧头的字节
     * @return 连续判断满足帧头条件则返回 true
     */
    bool FrameheaderSniffing(uint8_t byte){
      constexpr uint8_t _Offset_Uint = 0x01;

      if (byte == _HEAD_FLAG[3] && _SniffingProgress == _Offset_Uint << 2){
        _SniffingProgress = _Offset_Uint << 3;
        return true;
      }
      else if (byte == _HEAD_FLAG[2] && _SniffingProgress == _Offset_Uint << 1){
        _SniffingProgress = _Offset_Uint << 2;
      }
      else if (byte == _HEAD_FLAG[1] && _SniffingProgress == _Offset_Uint << 0){
        _SniffingProgress = _Offset_Uint << 1;
      }
      else if (byte == _HEAD_FLAG[0]){
        _SniffingProgress = _Offset_Uint;
      }
      else{
        _SniffingProgress = 0;
      }

      return false;
    }

    /**
     * @brief 初始化所有，不用调用，内部会自动调用
     * @return 成功 true 失败 false
     */
    bool init(){
      receivedBufferLength = 0;
      _enableCache = false;

      return true;
    }
  };

  /**
   * @class CRCBase
   * @brief 通用的CRC（循环冗余校验）基类。
   * 
   * 提供灵活的CRC计算功能，包括多项式、初始值、异或输出值、位反射支持等。
   * 通过派生子类，可以方便地实现常见的CRC算法。
   * 
   * 参数说明：
   * - polynomial: 用于计算CRC的多项式。
   * - initial_value: 初始化时的CRC值。
   * - xor_out_value: 最终输出的异或值。
   * - reflect_in: 是否对输入数据字节进行位反射。
   * - reflect_out: 是否对最终计算结果进行位反射。
   * - width: CRC的位宽（如8位、16位等）。
   */
  class CRCBase {
  public:
    CRCBase(uint32_t poly, uint32_t init, uint32_t xor_out, bool reflect_in,
            bool reflect_out, uint8_t width)
        : polynomial(poly), initial_value(init), xor_out_value(xor_out),
          reflect_in(reflect_in), reflect_out(reflect_out), width(width) {
      createTable();
    }

    virtual uint32_t crc(const uint8_t *bytes, size_t size) {
      uint32_t crc = initial_value;
      for (size_t i = 0; i < size; ++i) {
        uint8_t byte = reflect_in ? reflect(bytes[i], 8) : bytes[i];
        crc = (crc >> 8) ^ table[(crc ^ byte) & 0xFF];
      }
      crc = reflect_out ? reflect(crc, width) : crc;
      return (crc ^ xor_out_value) & ((1UL << width) - 1);
    }

  protected:
    uint32_t polynomial;
    uint32_t initial_value;
    uint32_t xor_out_value;
    bool reflect_in;
    bool reflect_out;
    uint8_t width;
    uint32_t table[256];

    void createTable() {
      for (uint16_t i = 0; i < 256; ++i) {
        uint32_t crc = i;
        for (uint8_t j = 0; j < 8; ++j) {
          if (crc & 1) {
            crc = (crc >> 1) ^ polynomial;
          } else {
            crc >>= 1;
          }
        }
        table[i] = crc;
      }
    }

    uint32_t reflect(uint32_t data, uint8_t bits) {
      uint32_t reflection = 0;
      for (uint8_t i = 0; i < bits; ++i) {
        if (data & (1 << i)) {
          reflection |= (1 << (bits - 1 - i));
        }
      }
      return reflection;
    }
  };

  /**
   * @class CRC8
   * @brief 实现CRC-8标准。
   * 
   * 标准配置：
   * - 多项式：0x07 (x^8 + x^2 + x + 1)
   * - 初始值：0x00
   * - 异或输出值：0x00
   * - 输入反射：否
   * - 输出反射：否
   * - 位宽：8位
   */
  class CRC8 : public CRCBase {
  public:
    CRC8() : CRCBase(0x07, 0x00, 0x00, false, false, 8) {}
    static constexpr uint32_t _CRC_LENGTH = 1;
  };

  /**
   * @class CRC16
   * @brief 实现CRC-16-MODBUS标准。
   * 
   * 标准配置：
   * - 多项式：0x8005 (x^16 + x^15 + x^2 + 1)
   * - 初始值：0x0000
   * - 异或输出值：0x0000
   * - 输入反射：是
   * - 输出反射：是
   * - 位宽：16位
   */
  class CRC16 : public CRCBase {
  public:
    CRC16() : CRCBase(0x8005, 0x0000, 0x0000, true, true, 16) {}
    static constexpr uint32_t _CRC_LENGTH = 2;
  };

  /**
   * @class CRC16_CCITT
   * @brief 实现CRC-16-CCITT标准（Kermit变体）。
   * 
   * 标准配置：
   * - 多项式：0x1021 (x^16 + x^12 + x^5 + 1)
   * - 初始值：0xFFFF
   * - 异或输出值：0x0000
   * - 输入反射：否
   * - 输出反射：否
   * - 位宽：16位
   */
  class CRC16_CCITT : public CRCBase {
  public:
      CRC16_CCITT() : CRCBase(0x1021, 0xFFFF, 0x0000, false, false, 16) {}
      static constexpr uint32_t _CRC_LENGTH = 2;
  };

  /**
   * @class CRC24
   * @brief 实现CRC-24标准。
   * 
   * 标准配置：
   * - 多项式：0x864CFB (x^24 + x^23 + x^6 + x^5 + x + 1)
   * - 初始值：0xB704CE
   * - 异或输出值：0x000000
   * - 输入反射：否
   * - 输出反射：否
   * - 位宽：24位
   */
  class CRC24 : public CRCBase {
  public:
    CRC24() : CRCBase(0x864CFB, 0xB704CE, 0x000000, false, false, 24) {}
    static constexpr uint32_t _CRC_LENGTH = 3;
  };

  /**
   * @class CRC32
   * @brief 实现CRC-32标准（IEEE 802.3）。
   * 
   * 标准配置：
   * - 多项式：0x04C11DB7 (x^32 + x^26 + x^23 + x^22 + x^16 + x^12 + x^11 + x^10 + x^8 + x^7 + x^5 + x^4 + x^2 + x + 1)
   * - 初始值：0xFFFFFFFF
   * - 异或输出值：0xFFFFFFFF
   * - 输入反射：是
   * - 输出反射：是
   * - 位宽：32位
   */
  class CRC32 : public CRCBase {
  public:
    CRC32() : CRCBase(0x04C11DB7, 0xFFFFFFFF, 0xFFFFFFFF, true, true, 32) {}
    static constexpr uint32_t _CRC_LENGTH = 4;
  };

}
