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
 * Simple C++ serial port driver (uses Boost ASIO).
 * ****************************************************************************
 */

#include <string>
#include <chrono>
#include <stdexcept>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <thread>
#include <mutex>
#include <condition_variable>

#include <SerialPort.hpp>

struct drivers::SerialPort::SerialPortPimpl
{
    SerialPortPimpl(const std::string& a_ttydev, int a_baudrate) : io(), port(io, a_ttydev), timer_rx(port.get_io_service()), timer_tx(port.get_io_service()), transfer_error_rx(true), transfer_error_tx(true)
    {
        if(!port.is_open())
        {
            throw std::runtime_error("Cannot open serial port");
        }
        
        // configure port
        port.set_option(boost::asio::serial_port_base::baud_rate(a_baudrate));
        port.set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
        port.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
        port.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
        port.set_option(boost::asio::serial_port_base::character_size(8));
    }
    
    ~SerialPortPimpl()
    {
        io.reset();
        timer_tx.cancel();
        timer_rx.cancel();
        port.close();
    }
    
    bool receiveBytes(std::size_t count, void* dst, int timeout) 
    {
        // After a timeout & cancel it seems we need to do a reset for subsequent reads to work.
        port.get_io_service().reset();
        
        if(timeout > 0)
        {
            // Asynchronously read data
            boost::asio::async_read(port, boost::asio::buffer(dst, count), boost::asio::transfer_at_least(count), boost::bind(&SerialPortPimpl::transfer_complete_rx, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred)); 

            // Setup a deadline time to implement our timeout.
            timer_rx.expires_from_now(boost::posix_time::milliseconds(timeout));
            timer_rx.async_wait(boost::bind(&SerialPortPimpl::time_out_rx, this, boost::asio::placeholders::error));
            
            // This will block until a character is read or until the it is cancelled.
            port.get_io_service().run();
            
            return transfer_error_rx;
        }
        else
        {
            return boost::asio::read(port, boost::asio::buffer(dst, count), boost::asio::transfer_at_least(count)) != count;
        }
    }
    
    bool transmitBytes(std::size_t count, const void* src, int timeout)
    {
        if(timeout > 0)
        {
            // After a timeout & cancel it seems we need to do a reset for subsequent writes to work.
            port.get_io_service().reset();
        
            // Asynchronously write data
            boost::asio::async_write(port, boost::asio::buffer(src, count), boost::asio::transfer_at_least(count), boost::bind(&SerialPortPimpl::transfer_complete_tx, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred)); 
            
            // Setup a deadline time to implement our timeout.
            timer_tx.expires_from_now(boost::posix_time::milliseconds(timeout));
            timer_tx.async_wait(boost::bind(&SerialPortPimpl::time_out_tx, this, boost::asio::placeholders::error));
            
            // This will block until a character is read or until the it is cancelled.
            port.get_io_service().run();
            
            return transfer_error_tx;
        }
        else if(timeout == 0)
        {
            return boost::asio::write(port, boost::asio::buffer(src, count), boost::asio::transfer_at_least(count)) != count;
        }
        else
        {
            // Asynchronously write data
            boost::asio::async_write(port, boost::asio::buffer(src, count), boost::asio::transfer_at_least(count), boost::bind(&SerialPortPimpl::transfer_complete_tx, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred)); 
            
            // Setup a deadline time to implement our timeout.
            timer_tx.expires_from_now(boost::posix_time::milliseconds(timeout));
            
            boost::system::error_code terr;
            timer_tx.wait(terr);
            
            if(terr)
            {
                return true; // timeout
            }
            
            return transfer_error_tx;
        }
    }
    
    // Called when an async operation completes or has been cancelled
    void transfer_complete_rx(const boost::system::error_code& error, size_t bytes_transferred) 
    {
        transfer_error_rx = (error || bytes_transferred == 0);

        // Read has finished, so cancel the timer.
        timer_rx.cancel();
    }
    
    // Called when the timer's deadline expires.
    void time_out_rx(const boost::system::error_code& error) 
    {
        // Was the timeout was cancelled?
        if (error) 
        {
            // yes
            return;
        }

        // no, we have timed out, so kill the read operation
        // The read callback will be called with an error
        port.cancel();
    }
    
    // Called when an async operation completes or has been cancelled
    void transfer_complete_tx(const boost::system::error_code& error, size_t bytes_transferred) 
    {
        transfer_error_tx = (error || bytes_transferred == 0);

        // Read has finished, so cancel the timer.
        timer_tx.cancel();
    }
    
    // Called when the timer's deadline expires.
    void time_out_tx(const boost::system::error_code& error) 
    {
        // Was the timeout was cancelled?
        if (error) 
        {
            // yes
            return;
        }
        
        // no, we have timed out, so kill the read operation
        // The read callback will be called with an error
        port.cancel();
    }
    
    boost::asio::io_service io;
    boost::asio::serial_port port;
    boost::asio::deadline_timer timer_rx, timer_tx;
    bool transfer_error_rx, transfer_error_tx;
};

drivers::SerialPort::SerialPort(const std::string& a_ttydev, int a_baudrate) : pimpl(new SerialPortPimpl(a_ttydev, a_baudrate))
{

}

drivers::SerialPort::~SerialPort()
{

}

bool drivers::SerialPort::receiveBytes(std::size_t count, void* dst, int timeout)
{
    return pimpl->receiveBytes(count, dst, timeout);
}

bool drivers::SerialPort::transmitBytes(std::size_t count, const void* src, int timeout)
{
    return pimpl->transmitBytes(count, src, timeout);
}
