/**
 * ****************************************************************************
 * Copyright (c) 2016, Robert Lukierski.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 * 
 * Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 * 
 * Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 * ****************************************************************************
 * Turtlebot Driver.
 * ****************************************************************************
 */
#ifndef TURTLEBOT_DRIVER_HPP
#define TURTLEBOT_DRIVER_HPP

#include <cstdint>
#include <cstddef>
#include <stdexcept>
#include <string>
#include <memory>
#include <functional>
#include <chrono>
#include <atomic>
#include <thread>

#include <misccpp/BufferEncode.hpp>
#include <misccpp/BufferDecode.hpp>

namespace drivers
{
    
class SerialPort;
    
namespace robot
{
    
class Turtlebot
{
public:
    static constexpr float RobotWheelRadius = 0.035f; // [m]
    static constexpr float RobotWheelBase = 0.260f; // [m]
    static constexpr float TickToRadian = 0.002436916871363930187454f;
    static constexpr float TickToMeter = 0.000085292090497737556558f;
    static constexpr float min_abs_yaw_vel = 0.0f;
    static constexpr float max_abs_yaw_vel = 0.0f;
    static constexpr float MaxWheelSpeed = 500.0f;
    
    class Exception : public std::exception
    {
    public:
        Exception() throw() { }

        virtual ~Exception() throw() { }
        virtual const char* what() const throw() { return "Turtlebot Exception"; }
    };
    
    enum class ChargingState : uint8_t
    {
        Discharging = 0,
        Docking_Charged = 2,
        Docking_Charging = 6,
        Adapter_Charged = 18,
        Adapter_Charging = 22
    };
    
    struct BasicSensorData
    {
        uint16_t        Timestamp;
        bool            BumperRight, BumperCentral, BumperLeft;
        bool            WheelDropRight, WheelDropLeft;
        bool            CliffRight, CliffCentral, CliffLeft;
        uint16_t        EncoderLeft, EncoderRight;
        int8_t          PWMLeft, PWMRight;
        bool            Button1, Button2, Button3;
        ChargingState   Charging;
        float           BatteryVoltage;
        bool            OvercurrentLeft, OvercurrentRight;
    };
    
    struct DockingIR
    {
        enum class State
        {
            NearLeft = 0x01,
            NearCenter = 0x02,
            NearRight = 0x04,
            FarCenter = 0x08,
            FarLeft = 0x10,
            FarRight = 0x20
        };
        
        State Left, Central, Right;
    };
    
    struct InertialSensor
    {
        float Angle; // ?
        float AngleRate; // ?
        uint8_t AccX, AccY, AccZ;
    };
    
    struct CliffSensor
    {
        float Right, Central, Left; // V
    };
    
    struct CurrentSensor
    {
        float Left, Right; // A
    };
    
    struct HardwareVersion
    {
        uint8_t Patch, Minor, Major;
    };
    
    struct FirmwareVersion
    {
        uint8_t Patch, Minor, Major;
    };
    
    struct RawGyroData
    {
        // TODO FIXME
    };
    
    struct GPIOStatus
    {
        bool GPIOInput0, GPIOInput1, GPIOInput2, GPIOInput3;
        float AnalogInput0, AnalogInput1, AnalogInput2, AnalogInput3;
    };
    
    struct UUID
    {
        uint32_t UUID0, UUID1, UUID2;
    };
    
    struct PIDController
    {
        bool FactorySettings;
        float Kp, Ki, Kd;
    };
    
    Turtlebot(const std::string& ttydev, int abaudrate, std::chrono::milliseconds to = std::chrono::milliseconds(1000));
    virtual ~Turtlebot();
    
    void motionStop();
    
    /**
     * @param rv rad/s.
     */
    void motionRotate(float rv);
    
    /**
     * @param speed mm/s.
     */
    void motionTranslate(float speed);
    
    /**
     * @param speed mm/s
     * @param radius mm
     */
    void motionCombined(float speed, float radius);
    
    /**
     * @param speed mm/s
     * @param ang_speed rad/s
     */
    void motionTwist(float speed, float ang_speed);
    
    void motionTwist2(float linear, float angular);
    
    /**
      * @param speed mm/s.
      * @param radius mm.
      */
    void setSpeedRadius(int16_t speed, int16_t radius);
    
    void requestHardwareVersion();
    void requestFirmwareVersion();
    void requestRobotUUID();
    
    void setGPIO(bool GPIOOut0 = false, bool GPIOOut1 = false, bool GPIOOut2 = false, bool GPIOOut3 = false,
                    bool ExtPower3V3 = false, bool ExtPower5V0 = false, bool ExtPower12V5A = false, bool ExtPower12V1A5 = false, 
                    bool LED1Red = false, bool LED1Green = false, bool LED2Red = false, bool LED2Green = false);
    
    void setPIDConfig(bool factory = true, float kp = 0.0f, float ki = 0.0f, float kd = 0.0f);
    
    std::function<void (const BasicSensorData&)> HandleBasicSensorData;
    std::function<void (const DockingIR&)> HandleDockingIR;
    std::function<void (const InertialSensor&)> HandleInertialSensor;
    std::function<void (const CliffSensor&)> HandleCliffSensor;
    std::function<void (const CurrentSensor&)> HandleCurrentSensor;
    std::function<void (const HardwareVersion&)> HandleHardwareVersion;
    std::function<void (const FirmwareVersion&)> HandleFirmwareVersion;
    std::function<void (const RawGyroData&)> HandleRawGyroData;
    std::function<void (const GPIOStatus&)> HandleGPIOStatus;
    std::function<void (const UUID&)> HandleUUID;
    std::function<void (const PIDController&)> HandlePIDController;
private:
    static constexpr uint8_t HeaderByte1 = 0xAA;
    static constexpr uint8_t HeaderByte2 = 0x55;
    
    enum class PayloadType : uint8_t
    {
        Header_BasicSensorData  = 0x01,
        Header_DockingIR        = 0x03,
        Header_InertialSensor   = 0x04,
        Header_CliffSensor      = 0x05,
        Header_CurrentSensor    = 0x06,
        Header_HardwareVersion  = 0x0A,
        Header_FirmwareVersion  = 0x0B,
        Header_RawGyroData      = 0x0D,
        Header_GPIOStatus       = 0x10,
        Header_UUID             = 0x13,
        Header_ControllerInfo   = 0x15
    };
    
    enum class CommandID : uint8_t
    {
        Command_BaseControl     = 0x01,
        Command_Sound           = 0x03,
        Command_SoundSequence   = 0x04,
        Command_RequestExtra    = 0x09,
        Command_SetGPIO         = 0x0C,
        Command_SetPIDConfig    = 0x0D,
        Command_GetPIDConfig    = 0x0E
    };
    
    enum class PortState
    {
        Waiting = 0,
        FirstByte,
        HeaderComplete,
        LengthDone,
        PayloadDone
    };
    
    void receiveThread();
    void sendPacket(BufferEncode& be);
    
    void parseBasicSensorData(BufferDecode& bd);
    void parseDockingIR(BufferDecode& bd);
    void parseInertialSensor(BufferDecode& bd);
    void parseCliffSensor(BufferDecode& bd);
    void parseCurrentSensor(BufferDecode& bd);
    void parseHardwareVersion(BufferDecode& bd);
    void parseFirmwareVersion(BufferDecode& bd);
    void parseRawGyroData(BufferDecode& bd);
    void parseGPIOStatus(BufferDecode& bd);
    void parseUUID(BufferDecode& bd);
    void parsePIDController(BufferDecode& bd);
    
    std::unique_ptr<SerialPort> sp;    
    std::atomic<bool> go_shutdown;
    std::thread poll_thread;
    std::array<uint8_t, 256> buffer_in, buffer_out;
    int timeoutMS;
};

}
    
}

#endif // TURTLEBOT_DRIVER_HPP
