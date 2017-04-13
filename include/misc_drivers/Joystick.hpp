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
 * Linux Joystick Wrapper.
 * ****************************************************************************
 */

#ifndef DRIVERS_JOYSTICK_HPP
#define DRIVERS_JOYSTICK_HPP

#include <cstdint>
#include <string>

namespace drivers
{
    
namespace sensor
{

class Joystick
{
public:    
    struct JoystickEvent
    {
        enum Type
        {
            Axis = 0,
            Button,
            Invalid
        };
        
        Type    Type;
        int16_t Index;
        int16_t Value;
    };
    
    Joystick(const std::string& devpath);
    ~Joystick();
    
    bool isConnected() const { return joy_fd != 0; }
    
    bool read(JoystickEvent& ev);
    
    uint32_t getVersion() const { return version; }
    uint8_t getAxisCount() const { return laxes; }
    uint8_t getButtonCount() const { return lbuttons; }
    std::string getName() const { return std::string(name); }
private:
    int joy_fd;
    uint32_t version;
    uint8_t laxes;
    uint8_t lbuttons;
    char name[256];
};

}

}

#endif // DRIVERS_JOYSTICK_HPP
