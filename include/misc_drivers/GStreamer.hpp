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
 * GStreamer C++ wrapper to build encoding pipelines.
 * ****************************************************************************
 */

#ifndef GSTREAMER_HPP
#define GSTREAMER_HPP

#include <stdint.h>
#include <stddef.h>
#include <stdexcept>
#include <cstring>
#include <string>
#include <sstream>
#include <memory>
#include <chrono>
#include <functional>
#include <thread>

typedef struct _GMainLoop GMainLoop;
typedef struct _GstElement GstElement;
typedef struct _GstBus GstBus;
typedef struct _GstMessage GstMessage;
typedef struct _GstBuffer GstBuffer;

namespace drivers
{

namespace gstreamer
{

class System
{
public:
    enum class LogLevel
    {
        NONE = 0,
        ERROR,
        WARNING,
        INFO,
        DEBUG,
        LOG
    };
    
    static System& get()
    {
        static System gs;
        return gs;
    }
    
    void setLogLevel(LogLevel ll);
    
    bool haveElementFactory(const std::string& name);
    
    void shutdown();
private:
    System();
    ~System();
    
    void mainLoop();
    std::thread loop_thr;
    GMainLoop* loop;
};

class AppSource;
class AppSink;
class Pipeline;

class Buffer
{
public:
    Buffer();
    Buffer(std::size_t len);
    Buffer(void* data, std::size_t len);
    ~Buffer();
    
    void* getData();
    std::size_t getSize();
    
    uint64_t getTimestamp();
    void setTimestamp(uint64_t ts);
    uint64_t getDuration();
    void setDuration(uint64_t ts);
    
    void setTimestampNone();
    void setDurationNone();
private:
    friend class Pipeline;
    friend class AppSource;
    friend class AppSink;
    
    GstBuffer* b;
};

class Pipeline
{
public:
    typedef std::function<bool(GstMessage*)> BusWatchT;
    
    Pipeline(const std::string& desc, bool watcher = false);
    virtual ~Pipeline();
    
    enum class State
    {
        VOID_PENDING = 0,
        IS_NULL      = 1,
        READY        = 2,
        PAUSED       = 3,
        PLAYING      = 4,
        TIMEOUT      = 10,
        ERROR        = 11
    };
    
    State getState();
    void setState(State s);
    
    BusWatchT BusWatchCallBack;
protected:
    virtual bool busWatch(GstMessage* msg) 
    { 
        if(BusWatchCallBack)
        {
            return BusWatchCallBack(msg);
        }
        else
        {
            return true;
        }
    }
    
private:
    friend class AppSource;
    friend class AppSink;
    static int busWatchKicker(GstBus* b, GstMessage* m, void* data);
    GstElement* pipeelem;
};

class AppSource
{
public:
    typedef std::function<void(unsigned int)> NeedDataT;
    typedef std::function<void()> EnoughDataT;
    
    AppSource(Pipeline& p, const std::string& name);
    virtual ~AppSource();
    
    virtual void needData(unsigned int size) 
    {
        if(NeedDataCallback)
        {
            NeedDataCallback(size);
        }
    }
    
    virtual void enoughData() 
    { 
        if(EnoughDataCallback)
        {
            EnoughDataCallback();
        }
    }
    
    Pipeline::State getState();
    bool pushBuffer(Buffer& b);
    
    NeedDataT NeedDataCallback;
    EnoughDataT EnoughDataCallback;
    
    void setEmitSignals(bool b);
    void setMaxBytes(std::size_t b);
    void setBlock(bool b);
    void setLive(bool b);
    void setDoTimestamp(bool b);
    void setBlocksize(std::size_t b);
protected:
    Pipeline& pipe;
    
private:
    static void needDataKicker(GstElement* pipeline, unsigned int size, void* data)
    {
        AppSource* cmp = (AppSource*)data;
        cmp->needData(size);
    }
    
    static void enoughDataKicker(GstElement* pipeline, void* data)
    {
        AppSource* cmp = (AppSource*)data;
        cmp->enoughData();
    }
    
    GstElement* source;
};

class AppSink
{
public:
    typedef std::function<bool()> NewBufferT;
    
    AppSink(Pipeline& p, const std::string& name);
    virtual ~AppSink();
    
    virtual bool newBuffer() 
    { 
        if(NewBufferCallback)
        {
            return NewBufferCallback();
        }
        else
        {
            return true; 
        }
    }
    
    Pipeline::State getState();
    bool pullBuffer(Buffer& b);
    
    NewBufferT NewBufferCallback;
    
    void setEmitSignals(bool b);
    void setSync(bool b);
    void setDrop(bool b);
protected:
    Pipeline& pipe;
    
private:
    static int newBufferKicker(GstElement* sink, void* data);
    
    GstElement* sink;
};

}

}

#endif // GSTREAMER_HPP
