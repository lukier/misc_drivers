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

#include <cmath>
#include <algorithm>
#include <misccpp/drivers/SerialPort.hpp>
#include <misc_drivers/TurtlebotDriver.hpp>

drivers::robot::Turtlebot::Turtlebot(const std::string& ttydev, int abaudrate, std::chrono::milliseconds to) : 
    sp(new SerialPort()), go_shutdown(false), timeoutMS(to.count())
{
    sp->open(ttydev.c_str(), abaudrate);
    poll_thread = std::thread(&drivers::robot::Turtlebot::receiveThread, this);
}

drivers::robot::Turtlebot::~Turtlebot()
{
    go_shutdown = true;
    poll_thread.join();
}

void drivers::robot::Turtlebot::receiveThread()
{
    PortState cstate = PortState::Waiting;
    uint8_t data = 0x00;
    uint8_t length = 0x00;
    uint8_t cnt = 0x00;
    bool ok = false;
    
    while(go_shutdown == false)
    {
        // TODO FIXME rewrite this state machine
        try
        {
            if(!sp->read(&data, 1, std::chrono::milliseconds(timeoutMS))) // timeout
            {
                continue;
            }
            
            // check first byte
            if((data == 0xAA) && (cstate == PortState::Waiting))
            {
                cstate = PortState::FirstByte;
                continue;
            }
            
            // check second byte
            if(cstate == PortState::FirstByte)
            {
                if(data == 0x55)
                {
                    cstate = PortState::HeaderComplete;
                }
                else // go back
                {
                    cstate = PortState::Waiting;
                }
                continue;
            }
            
            // read length
            if(cstate == PortState::HeaderComplete)
            {
                length = data;
                
                // read payload
                if(!sp->read(buffer_in.begin(), length, std::chrono::milliseconds(timeoutMS))) // timeout
                {
                    cstate = PortState::Waiting;
                    continue;
                }
                
                cstate = PortState::PayloadDone;
                continue;
            }
                        
            // read checksum and check
            if(cstate == PortState::PayloadDone)
            {
                uint8_t data_cs = 0;
                
                data_cs ^= length;
                
                for(uint8_t i = 0 ; i < length ; ++i)
                {
                    data_cs ^= buffer_in[i];
                }
                
                if(data_cs == data) // checksum OK
                {
                    int pos = 0;
                    
                    while(1)
                    {
                        PayloadType header = (PayloadType)buffer_in[pos];
                        ++pos;
                        int splen = (int)buffer_in[pos];
                        ++pos;
                        
                        BufferDecode bd(&buffer_in[pos], splen);
                        switch(header)
                        {
                            case PayloadType::Header_BasicSensorData:
                                parseBasicSensorData(bd);
                                break;
                            case PayloadType::Header_DockingIR:
                                parseDockingIR(bd);
                                break;
                            case PayloadType::Header_InertialSensor:
                                parseInertialSensor(bd);
                                break;
                            case PayloadType::Header_CliffSensor:
                                parseCliffSensor(bd);
                                break;
                            case PayloadType::Header_CurrentSensor:
                                parseCurrentSensor(bd);
                                break;
                            case PayloadType::Header_HardwareVersion:
                                parseHardwareVersion(bd);
                                break;
                            case PayloadType::Header_FirmwareVersion:
                                parseFirmwareVersion(bd);
                                break;
                            case PayloadType::Header_RawGyroData:
                                parseRawGyroData(bd);
                                break;
                            case PayloadType::Header_GPIOStatus:
                                parseGPIOStatus(bd);
                                break;
                            case PayloadType::Header_UUID:
                                parseUUID(bd);
                                break;
                            case PayloadType::Header_ControllerInfo:
                                parsePIDController(bd);
                                break;
                            default:
                                throw std::runtime_error("Unknown packet type");
                                break;
                        }
                        
                        pos += splen;
                        if(pos >= length - 1)
                        {
                            break;
                        }
                    }
                }
                else
                {
                    throw std::runtime_error("Wrong checksum");
                }
                
                cstate = PortState::Waiting;
            }
        }
        catch(const std::exception& ex)
        {
            // TODO FIXME exception in the receiving thread
        }
    }
}

void drivers::robot::Turtlebot::parseBasicSensorData(BufferDecode& bd)
{
    uint8_t byte = 0;
    BasicSensorData bsd;
    
    // timestamp
    bd.read(bsd.Timestamp);
    
    // bumpers
    bd.read(byte);
    if((byte & 0x01) != 0) { bsd.BumperRight = true; } else { bsd.BumperRight = false; }
    if((byte & 0x02) != 0) { bsd.BumperCentral = true; } else { bsd.BumperCentral = false; }
    if((byte & 0x04) != 0) { bsd.BumperLeft = true; } else { bsd.BumperLeft = false; }
    
    // wheel drop
    bd.read(byte);
    if((byte & 0x01) != 0) { bsd.WheelDropRight = true; } else { bsd.WheelDropRight = false; }
    if((byte & 0x02) != 0) { bsd.WheelDropLeft = true; } else { bsd.WheelDropLeft = false; }
    
    // cliff
    bd.read(byte);
    if((byte & 0x01) != 0) { bsd.CliffRight = true; } else { bsd.CliffRight = false; }
    if((byte & 0x02) != 0) { bsd.CliffCentral = true; } else { bsd.CliffCentral = false; }
    if((byte & 0x04) != 0) { bsd.CliffLeft = true; } else { bsd.CliffLeft = false; }
    
    // left encoder
    bd.read(bsd.EncoderLeft);
    
    // right encoder
    bd.read(bsd.EncoderRight);
    
    // left PWM
    bd.read(bsd.PWMLeft);
    
    // right PWM
    bd.read(bsd.PWMRight);
    
    // buttons
    bd.read(byte);
    if((byte & 0x01) != 0) { bsd.Button1 = true; } else { bsd.Button1 = false; }
    if((byte & 0x02) != 0) { bsd.Button2 = true; } else { bsd.Button2 = false; }
    if((byte & 0x04) != 0) { bsd.Button3 = true; } else { bsd.Button3 = false; }
    
    // charging state
    bd.read(byte);
    bsd.Charging = (ChargingState)byte;
    
    // voltage
    bd.read(byte);
    bsd.BatteryVoltage = (float)byte * 0.1f; 
    
    // overcurrent
    bd.read(byte);
    if((byte & 0x01) != 0) { bsd.OvercurrentLeft = true; } else { bsd.OvercurrentLeft = false; }
    if((byte & 0x02) != 0) { bsd.OvercurrentRight = true; } else { bsd.OvercurrentRight = false; }
    
    if(HandleBasicSensorData)
    {
        HandleBasicSensorData(bsd);
    }
}

void drivers::robot::Turtlebot::parseDockingIR(BufferDecode& bd)
{
    uint8_t byte = 0;
    DockingIR dir;
    
    bd.read(byte);
    dir.Right = (DockingIR::State)byte;
    bd.read(byte);
    dir.Central = (DockingIR::State)byte;
    bd.read(byte);
    dir.Left = (DockingIR::State)byte;
    
    if(HandleDockingIR)
    {
        HandleDockingIR(dir);
    }
}

void drivers::robot::Turtlebot::parseInertialSensor(BufferDecode& bd)
{
    int16_t data = 0;
    InertialSensor is;
    
    // raw data angles are in hundredths of a degree, convert to radians.
    bd.read(data);
    is.Angle = (data / 100.0f) * (M_PI/180.0f); 
    
    bd.read(data);
    is.AngleRate = (data / 100.0f) * (M_PI/180.0f);
    
    bd.read(is.AccX);
    bd.read(is.AccY);
    bd.read(is.AccZ);
    
    if(HandleInertialSensor)
    {
        HandleInertialSensor(is);
    }
}

void drivers::robot::Turtlebot::parseCliffSensor(BufferDecode& bd)
{
    uint16_t data = 0;
    CliffSensor cs;
    
    bd.read(data);
    cs.Right = ((float)data * 3.3f) / 4095.0f; // V
    bd.read(data);
    cs.Central = ((float)data * 3.3f) / 4095.0f; // V
    bd.read(data);
    cs.Left = ((float)data * 3.3f) / 4095.0f; // V
    
    if(HandleCliffSensor)
    {
        HandleCliffSensor(cs);
    }
}

void drivers::robot::Turtlebot::parseCurrentSensor(BufferDecode& bd)
{
    uint8_t data = 0;
    CurrentSensor cs;
    
    bd.read(data);
    cs.Left = (float)data * 0.01f; // A
    
    bd.read(data);
    cs.Right = (float)data * 0.01f; // A
    
    if(HandleCurrentSensor)
    {
        HandleCurrentSensor(cs);
    }
}

void drivers::robot::Turtlebot::parseHardwareVersion(BufferDecode& bd)
{
    HardwareVersion hv;
    
    bd.read(hv.Patch);
    bd.read(hv.Minor);
    bd.read(hv.Major);
    // unused
    
    if(HandleHardwareVersion)
    {
        HandleHardwareVersion(hv);
    }
}

void drivers::robot::Turtlebot::parseFirmwareVersion(BufferDecode& bd)
{
    FirmwareVersion fv;
    
    bd.read(fv.Patch);
    bd.read(fv.Minor);
    bd.read(fv.Major);
    // unused
    
    if(HandleFirmwareVersion)
    {
        HandleFirmwareVersion(fv);
    }
}

void drivers::robot::Turtlebot::parseRawGyroData(BufferDecode& bd)
{
    RawGyroData rgd;
    
    // TODO FIXME
    
    if(HandleRawGyroData)
    {
        HandleRawGyroData(rgd);
    }
}

void drivers::robot::Turtlebot::parseGPIOStatus(BufferDecode& bd)
{
    uint16_t data = 0;
    GPIOStatus gs;
    
    bd.read(data);
    if((data & 0x0001) != 0) { gs.GPIOInput0 = true; } else { gs.GPIOInput0 = false; }
    if((data & 0x0002) != 0) { gs.GPIOInput1 = true; } else { gs.GPIOInput1 = false; }
    if((data & 0x0004) != 0) { gs.GPIOInput2 = true; } else { gs.GPIOInput2 = false; }
    if((data & 0x0008) != 0) { gs.GPIOInput3 = true; } else { gs.GPIOInput3 = false; }
    
    bd.read(data);
    gs.AnalogInput0 = ((float)data * 3.3f) / 4095.0f; // V
    
    bd.read(data);
    gs.AnalogInput1 = ((float)data * 3.3f) / 4095.0f; // V
    
    bd.read(data);
    gs.AnalogInput2 = ((float)data * 3.3f) / 4095.0f; // V
    
    bd.read(data);
    gs.AnalogInput3 = ((float)data * 3.3f) / 4095.0f; // V
    
    if(HandleGPIOStatus)
    {
        HandleGPIOStatus(gs);
    }
}

void drivers::robot::Turtlebot::parseUUID(BufferDecode& bd)
{
    UUID uid;
    bd.read(uid.UUID0);
    bd.read(uid.UUID1);
    bd.read(uid.UUID2);
    
    if(HandleUUID)
    {
        HandleUUID(uid);
    }
}

void drivers::robot::Turtlebot::parsePIDController(BufferDecode& bd)
{
    uint8_t byte = 0;
    uint32_t data = 0;
    PIDController pid;
    
    bd.read(byte);
    if(byte == 0) { pid.FactorySettings = true; } else { pid.FactorySettings = false; }
    
    bd.read(data);
    pid.Kp = ((float)data) / 1000.0f;
    
    bd.read(data);
    pid.Ki = ((float)data) / 1000.0f;
    
    bd.read(data);
    pid.Kd = ((float)data) / 1000.0f;
    
    if(HandlePIDController)
    {
        HandlePIDController(pid);
    }
}

void drivers::robot::Turtlebot::sendPacket(BufferEncode& be)
{
    // 2 bytes of header, 1 byte length, payload
    const std::size_t total_packet_size = 2 + 1 + be.getCurrentSize();
    
    // header
    buffer_out[0] = HeaderByte1;
    buffer_out[1] = HeaderByte2;
    
    // length
    buffer_out[2] = be.getCurrentSize();
    
    // payload
    be.encodeTo(&buffer_out[3]);
    
    // calculate and write checksum
    uint8_t checksum = buffer_out[2];
    for(std::size_t i = 2 ; i < total_packet_size ; ++i)
    {
        checksum ^= buffer_out[i];
    }
    buffer_out[total_packet_size] = checksum;
    
    // transmit
    sp->write(buffer_out.begin(), total_packet_size + 1, std::chrono::milliseconds(0));
}

void drivers::robot::Turtlebot::motionStop()
{
    setSpeedRadius(0,0);
}

void drivers::robot::Turtlebot::motionTranslate(float speed)
{
    setSpeedRadius(speed * 1000.0f, 0);
}

void drivers::robot::Turtlebot::motionRotate(float rv)
{
    const int16_t spd = (rv * (RobotWheelBase * 1000.0f)) / 2.0f;
    
    setSpeedRadius(spd, 1);
}

void drivers::robot::Turtlebot::motionCombined(float speed, float radius)
{
    float outspd = 0.0f;
    
    if(radius > 1.0f)
    {
        outspd = speed * ( radius + (RobotWheelBase * 1000.0f) / 2.0f ) / radius;
    }
    else if(radius < -1.0f)
    {
        outspd = speed * ( radius - (RobotWheelBase * 1000.0f) / 2.0f ) / radius;
    }
    else
    {
        // straight line
        radius = 0.0f;
        outspd = speed * (RobotWheelBase * 1000.0f / 2.0f);
    }
    
    setSpeedRadius(outspd, radius);
}

void drivers::robot::Turtlebot::motionTwist(float speed, float ang_speed)
{
    float radius = 0.0f;
    const float epsilon = 0.0001f;
    
    // Special Case #1 : Straight Run
    if(fabs(ang_speed) < epsilon ) 
    {
        radius = 0.0f;
        setSpeedRadius(speed * 1000.0f, radius);
        return;
    }
    
    radius = speed * 1000.0f / ang_speed;
    
    // Special Case #2 : Pure Rotation or Radius is less than or equal to 1.0 mm
    if( fabs(speed) < epsilon || fabs(radius) <= 0.001f ) 
    {
        speed  = (RobotWheelBase * 1000.0f) * ang_speed / 2.0f;
        radius = 1.0f;
        setSpeedRadius(speed, radius);
        return;
    }
    
    // General Case :
    if( radius > 0.0f ) 
    {
        speed  = (radius + (RobotWheelBase * 1000.0f) / 2.0f) * ang_speed;
    } 
    else 
    {
        speed  = (radius - (RobotWheelBase * 1000.0f) / 2.0f) * ang_speed;
    }
    
    setSpeedRadius(speed, radius);
}

void drivers::robot::Turtlebot::motionTwist2(float linear, float angular)
{
    // Clamp to min abs yaw velocity, to avoid trying to rotate at low  speeds, which doesn't work well.
    if((min_abs_yaw_vel > 0.0f) && (fabs(angular) >= std::numeric_limits<float>::epsilon()) && (fabs(angular) < min_abs_yaw_vel))
    {
        if(angular > 0.0f)
        {
            angular = min_abs_yaw_vel;
        }
        else
        {
            angular = -min_abs_yaw_vel;
        }
    }
    
    // Limit maximum yaw to avoid saturating the gyro
    if((max_abs_yaw_vel > 0.0f) && (fabs(angular) >= std::numeric_limits<float>::epsilon()) && (fabs(angular) > max_abs_yaw_vel))
    {
        if(angular > 0.0f)
        {
            angular = max_abs_yaw_vel;
        }
        else
        {
            angular = -max_abs_yaw_vel;
        }
    }
    
    float ts  = linear * 1000.0f;
    const float tw  = angular  * (RobotWheelBase / 2.0f) * 1000.0f;
    
    if(ts > 0.0f)
    {
        ts = std::min(ts, MaxWheelSpeed - (float)fabs(tw));
    }
    else
    {
        ts = std::max(ts, -(MaxWheelSpeed - (float)fabs(tw)));
    }
    
    setSpeedRadius(int16_t(ts - tw), int16_t(ts + tw));
}

void drivers::robot::Turtlebot::setSpeedRadius(int16_t speed, int16_t radius)
{
    BufferEncode be;
    const uint8_t command_id = (uint8_t)CommandID::Command_BaseControl;
    const uint8_t len = 0x04;
    be.write(&command_id);
    be.write(&len);
    be.write(&speed);
    be.write(&radius);
    sendPacket(be);
}

void drivers::robot::Turtlebot::requestHardwareVersion()
{
    BufferEncode be;
    const uint8_t command_id = (uint8_t)CommandID::Command_RequestExtra;
    const uint8_t len = 0x02;
    const uint8_t val = 0x01;
    be.write(&command_id);
    be.write(&len);
    be.write(&val);
    sendPacket(be);
}

void drivers::robot::Turtlebot::requestFirmwareVersion()
{
    BufferEncode be;
    const uint8_t command_id = (uint8_t)CommandID::Command_RequestExtra;
    const uint8_t len = 0x02;
    const uint8_t val = 0x02;
    be.write(&command_id);
    be.write(&len);
    be.write(&val);
    sendPacket(be);
}

void drivers::robot::Turtlebot::requestRobotUUID()
{
    BufferEncode be;
    const uint8_t command_id = (uint8_t)CommandID::Command_RequestExtra;
    const uint8_t len = 0x02;
    const uint8_t val = 0x08;
    be.write(&command_id);
    be.write(&len);
    be.write(&val);
    sendPacket(be);
}

void drivers::robot::Turtlebot::setGPIO(bool GPIOOut0, bool GPIOOut1, bool GPIOOut2, bool GPIOOut3, 
                             bool ExtPower3V3, bool ExtPower5V0, bool ExtPower12V5A, bool ExtPower12V1A5, 
                             bool LED1Red, bool LED1Green, bool LED2Red, bool LED2Green)
{
    BufferEncode be;
    const uint8_t command_id = (uint8_t)CommandID::Command_SetGPIO;
    const uint8_t len = 0x02;
    uint16_t val = 0x0000;
    
    if(GPIOOut0) { val |= 0x0001; }
    if(GPIOOut1) { val |= 0x0002; }
    if(GPIOOut2) { val |= 0x0004; }
    if(GPIOOut3) { val |= 0x0008; }
    
    if(ExtPower3V3) { val |= 0x0010; }
    if(ExtPower5V0) { val |= 0x0020; }
    if(ExtPower12V5A) { val |= 0x0040; }
    if(ExtPower12V1A5) { val |= 0x0080; }
    
    if(LED1Red) { val |= 0x0100; }
    if(LED1Green) { val |= 0x0200; }
    if(LED2Red) { val |= 0x0400; }
    if(LED2Green) { val |= 0x0800; }
    
    be.write(&command_id);
    be.write(&len);
    be.write(&val);
    sendPacket(be);
}

void drivers::robot::Turtlebot::setPIDConfig(bool factory, float kp, float ki, float kd)
{
    BufferEncode be;
    const uint8_t command_id = (uint8_t)CommandID::Command_SetPIDConfig;
    const uint8_t len = 0x0D;
    const uint8_t mode = (factory == true ? 0 : 1);
    const uint32_t ikp = (uint32_t)(kp * 1000.0f);
    const uint32_t iki = (uint32_t)(ki * 1000.0f);
    const uint32_t ikd = (uint32_t)(kd * 1000.0f);
    be.write(&command_id);
    be.write(&len);
    be.write(&mode);
    be.write(&ikp);
    be.write(&iki);
    be.write(&ikd);
    sendPacket(be);
}
