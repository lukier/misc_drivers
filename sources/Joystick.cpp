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

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <linux/joystick.h>

#include <stdexcept>

#include <misc_drivers/Joystick.hpp>

drivers::sensor::Joystick::Joystick(const std::string& devpath) : joy_fd(0)
{
    joy_fd = ::open(devpath.c_str(), O_RDONLY | O_NONBLOCK);
    
    if(joy_fd > 0) 
    {
        ::ioctl(joy_fd, JSIOCGNAME(256), name);
        ::ioctl(joy_fd, JSIOCGVERSION, &version);
        ::ioctl(joy_fd, JSIOCGAXES, &laxes);
        ::ioctl(joy_fd, JSIOCGBUTTONS, &lbuttons);
    }
    else
    {
        throw std::runtime_error("Cannot open joystick");
    }
}

drivers::sensor::Joystick::~Joystick()
{
    // close joystick stuff
    if(joy_fd != 0)
    {
        ::close(joy_fd);
        joy_fd = 0;
    }
}

bool drivers::sensor::Joystick::read(JoystickEvent& ev)
{
    struct js_event joystick_ev;
    
    int bytes = ::read(joy_fd, &joystick_ev, sizeof(js_event));
    if(bytes != sizeof(js_event))
    {
        return false;
    }
    
    joystick_ev.type &= ~JS_EVENT_INIT;
    
    ev.Type = JoystickEvent::Invalid;
    ev.Index = joystick_ev.number;
    ev.Value = joystick_ev.value;
    
    if(joystick_ev.type & JS_EVENT_AXIS)
    {
        ev.Type = JoystickEvent::Axis;
    }
    else if(joystick_ev.type & JS_EVENT_BUTTON)
    {
        ev.Type = JoystickEvent::Button;
    }
    
    return true;
}
