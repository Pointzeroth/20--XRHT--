#pragma once

#include "common.hpp"
#include <iostream>               // 输入输出类
#include <libserial/SerialPort.h> // 串口通信
#include <math.h>                 // 数学函数类
#include <stdint.h>               // 整型数据类
#include <string.h>
#include <thread>

using namespace LibSerial;
using namespace std;

// USB通信帧
#define USB_FRAME_HEAD 0xFE // USB通信帧头
#define USB_FRAME_END 0xFF

class Uart 
{
private:
  /**
   * @brief 串口通信结构体
   *
   */
  typedef struct 
  {
    bool start;                           // 开始接收标志
    int buffRead[3];   // 临时缓冲数据
  } SerialStruct;

  std::unique_ptr<std::thread> threadRec; // 串口接收子线程
  std::shared_ptr<SerialPort> serialPort = nullptr;
  std::string portName; // 端口名字
  bool isOpen = false;
  SerialStruct serialStr; // 串口通信数据结构体

  /**
   * @brief 串口接收字节数据
   *
   * @param charBuffer
   * @param msTimeout
   * @return int
   */
  int receiveBytes(unsigned char &charBuffer, size_t msTimeout = 0) 
  {
    /*try检测语句块有没有异常。如果没有发生异常,就检测不到。
    如果发生异常，則交给 catch 处理，执行 catch 中的语句* */
    try {
      /*从串口读取一个数据,指定msTimeout时长内,没有收到数据，抛出异常。
      如果msTimeout为0，则该方法将阻塞，直到数据可用为止。*/
      serialPort->ReadByte(charBuffer, msTimeout); // 可能出现异常的代码段
    } catch (const ReadTimeout &) // catch捕获并处理 try 检测到的异常。
    {
      // std::cerr << "The ReadByte() call has timed out." << std::endl;
      return -2;
    } catch (const NotOpen &) // catch()中指明了当前 catch 可以处理的异常类型
    {
      std::cerr << "Port Not Open ..." << std::endl;
      return -1;
    }
    return 0;
  };

  /**
   * @brief
   *
   * @param data
   * @return int
   */
  int transmitByte(unsigned char data) 
  {
    // try检测语句块有没有异常
    try {
      serialPort->WriteByte(data); // 写数据到串口
    } catch (const std::runtime_error &) // catch捕获并处理 try 检测到的异常。
    {
      std::cerr << "The Write() runtime_error." << std::endl;
      return -2;
    } catch (const NotOpen &) // catch捕获并处理 try 检测到的异常。
    {
      std::cerr << "Port Not Open ..." << std::endl;
      return -1;
    }
    serialPort->DrainWriteBuffer(); // 等待，直到写缓冲区耗尽，然后返回。
    return 0;
  }

public:
  // 定义构造函数
  Uart(const std::string &port) : portName(port){};
  // 定义析构函数
  ~Uart() { close(); };
  bool keypress = false; // 按键

  /**
   * @brief 蜂鸣器音效
   *
   */
  enum Buzzer 
  {
    BUZZER_OK = 0,   // 确认
    BUZZER_WARNNING, // 报警
    BUZZER_FINISH,   // 完成
    BUZZER_DING,     // 提示
    BUZZER_START,    // 开机
  };

public:

  /**
   * @brief 启动串口通信
   *
   * @param port 串口号
   * @return int
   */
  int open(void) 
  {
    serialPort = std::make_shared<SerialPort>();
    if (serialPort == nullptr) {
      std::cerr << "Serial Create Failed ." << std::endl;
      return -1;
    }
    // try检测语句块有没有异常
    try {
      serialPort->Open(portName);                     // 打开串口
      serialPort->SetBaudRate(BaudRate::BAUD_115200); // 设置波特率
      serialPort->SetCharacterSize(CharacterSize::CHAR_SIZE_8); // 8位数据位
      serialPort->SetFlowControl(FlowControl::FLOW_CONTROL_NONE); // 设置流控
      serialPort->SetParity(Parity::PARITY_NONE);                 // 无校验
      serialPort->SetStopBits(StopBits::STOP_BITS_1); // 1个停止位
    } catch (const OpenFailed &) // catch捕获并处理 try 检测到的异常。
    {
      std::cerr << "Serial port: " << portName << "open failed ..."
                << std::endl;
      isOpen = false;
      return -2;
    } catch (const AlreadyOpen &) // catch捕获并处理 try 检测到的异常。
    {
      std::cerr << "Serial port: " << portName << "open failed ..."
                << std::endl;
      isOpen = false;
      return -3;
    } catch (...) // catch捕获并处理 try 检测到的异常。
    {
      std::cerr << "Serial port: " << portName << " recv exception ..."
                << std::endl;
      isOpen = false;
      return -4;
    }

    serialStr.start = false;
    isOpen = true;

    return 0;
  }

  /**
   * @brief 启动接收子线程
   *
   */
  void startReceive(void) 
  {
    if (!isOpen) // 串口是否正常打开
      return;

    // 启动串口接收子线程
    threadRec = std::make_unique<std::thread>([this]() 
    {
      while (1) 
      {
        receiveCheck(); // 串口接收校验
      }
    });
  }

  /**
   * @brief 关闭串口通信
   *
   */
  void close(void) 
  {
    printf(" uart thread exit!\n");
    carControl(0, 0);
    // threadRec->join();
    if (serialPort != nullptr) {
      serialPort->Close();
      serialPort = nullptr;
    }
    isOpen = false;
  }

  /**
   * @brief 串口接收校验
   *
   */
  void receiveCheck(void) 
  {
    if (!isOpen) // 串口是否正常打开
      return;

    uint8_t resByte = 0;
    int ret = receiveBytes(resByte, 0);
    if (ret == 0) 
    {
      if (resByte == 0x42 && !serialStr.start) // 监听帧头
      {
        serialStr.start = true;                   // 开始接收数据
        serialStr.buffRead[0] = resByte;          // 获取帧头
      }
      else if (resByte == 0x43 && serialStr.start) // 帧长接收完毕
      {
        serialStr.start = false; // 重新监听帧头
        serialStr.buffRead[2] = resByte;
      }
      else if (serialStr.start) // 开始接收数据
      {
        keypress = true;
        serialStr.buffRead[1] = resByte; // 读取数据
        // std::cout << "receive:" << resByte << std::endl;
      }
    }
  }
  
  /**
   * @brief 速度+方向控制
   *
   * @param speed 速度：m/s
   * @param servo 方向：PWM（500~2500）
   */
  void carControl(int servo, int speed) 
  {
    if (!isOpen)
      return;
    if(servo >= 350)
        servo = 350;
    if(servo <= -350)
        servo = -350;

    uint8_t buff[8];  // 多发送一个字节

    buff[0] = USB_FRAME_HEAD;   // 通信帧头
    buff[1] = static_cast<uint8_t>(servo < 0);
    buff[2] = static_cast<uint8_t>(abs(speed) / 10);
    buff[3] = static_cast<uint8_t>(abs(speed) % 10);
    buff[4] = static_cast<uint8_t>(abs(servo) >> 8);
    buff[5] = static_cast<uint8_t>(abs(servo) & 0xFF);
    buff[6] = static_cast<uint8_t>(speed < 0);
    buff[7] = USB_FRAME_END;

    // 循环发送数据
    for (size_t i = 0; i < 8; i++)
      transmitByte(buff[i]);
  }
};
