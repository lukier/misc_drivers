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

#include <GStreamer.hpp>

// Not my code, not my problem
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wall"
#pragma GCC diagnostic ignored "-Wcpp"
#pragma GCC diagnostic ignored "-Wliteral-suffix"
#define GST_USE_UNSTABLE_API
#include <gst/gst.h>
#include <gst/video/gstbasevideocodec.h>
#include <gst/app/gstappsrc.h>
#include <gst/app/gstappsink.h>
#include <gst/gstinfo.h>
#pragma GCC diagnostic pop

drivers::gstreamer::System::System()
{
    if(gst_is_initialized() == FALSE)
    {
        gst_init(NULL, NULL);
    }
    
    //loop = g_main_loop_new (NULL, FALSE);
    
    //loop_thr = std::thread(&drivers::gstreamer::System::mainLoop, this);
}

drivers::gstreamer::System::~System()
{
    shutdown();
}

void drivers::gstreamer::System::mainLoop()
{
    //g_main_loop_run(loop);
}

void drivers::gstreamer::System::setLogLevel(drivers::gstreamer::System::LogLevel ll)
{
    switch(ll)
    {
        case LogLevel::NONE: gst_debug_set_default_threshold(GST_LEVEL_NONE); break;
        case LogLevel::ERROR: gst_debug_set_default_threshold(GST_LEVEL_ERROR); break;
        case LogLevel::WARNING: gst_debug_set_default_threshold(GST_LEVEL_WARNING); break;
        case LogLevel::INFO: gst_debug_set_default_threshold(GST_LEVEL_INFO); break;
        case LogLevel::DEBUG: gst_debug_set_default_threshold(GST_LEVEL_DEBUG); break;
        case LogLevel::LOG: gst_debug_set_default_threshold(GST_LEVEL_LOG); break;
        default: gst_debug_set_default_threshold(GST_LEVEL_NONE); break;
    }
}

bool drivers::gstreamer::System::haveElementFactory(const std::string& name)
{
    bool ret = false;
    GstElementFactory *factory = NULL;
    
    /* get factory */
    factory = gst_element_factory_find (name.c_str());
    if(factory) 
    {
        gst_object_unref (GST_OBJECT (factory));
        ret = true; 
    }
    return ret;
}

void drivers::gstreamer::System::shutdown()
{
    if(gst_is_initialized() == TRUE)
    {
        //g_main_loop_quit(loop);
        
        //loop_thr.join();
        
        gst_deinit();
    }
}

drivers::gstreamer::Pipeline::Pipeline(const std::string& desc, bool watcher)
{
    System::get();
    
    GError *error = NULL;
    
    pipeelem = gst_parse_launch(desc.c_str(), &error);
    
    if(pipeelem == NULL)
    {
        throw std::runtime_error("Error creating the pipeline");
    }
    
    if(watcher)
    {
        GstBus* bus = gst_pipeline_get_bus(GST_PIPELINE(pipeelem));
        gst_bus_add_watch(bus, (GstBusFunc)busWatchKicker, this);
        gst_object_unref(bus);
    }
}

drivers::gstreamer::Pipeline::~Pipeline()
{
    gst_object_unref (GST_OBJECT (pipeelem));
}

int drivers::gstreamer::Pipeline::busWatchKicker(GstBus* b, GstMessage* m, void* data)
{
    drivers::gstreamer::Pipeline* ppl = (drivers::gstreamer::Pipeline*)data;
    bool ret = ppl->busWatch(m);
    if(ret)
    {
        return TRUE;
    }
    else
    {
        return FALSE;
    }
}

drivers::gstreamer::Pipeline::State drivers::gstreamer::Pipeline::getState()
{
    //uint64_t timeout = GST_CLOCK_TIME_NONE;
    uint64_t timeout = 1000000; // 1 ms
    GstState state; 
    GstStateChangeReturn gscr = gst_element_get_state(GST_ELEMENT (pipeelem), &state, NULL, timeout);
    if(gscr != GST_STATE_CHANGE_SUCCESS)
    {
        return Pipeline::State::ERROR;
    }
    else
    {
        return (drivers::gstreamer::Pipeline::State)state;
    }
}

void drivers::gstreamer::Pipeline::setState(drivers::gstreamer::Pipeline::State s)
{
    gst_element_set_state(GST_ELEMENT(pipeelem), (GstState)s);
}

drivers::gstreamer::Buffer::Buffer()
{
    System::get();
}

drivers::gstreamer::Buffer::Buffer(std::size_t len)
{
    System::get();
    
    b = gst_buffer_new_and_alloc(len);
}

drivers::gstreamer::Buffer::Buffer(void* data, std::size_t len)
{
    System::get();
    
    b = gst_buffer_new();
    gst_buffer_set_data(b, (guint8*)data, len);
}

drivers::gstreamer::Buffer::~Buffer()
{
    gst_buffer_unref(b);
}

void* drivers::gstreamer::Buffer::getData() { return GST_BUFFER_DATA(b); }
std::size_t drivers::gstreamer::Buffer::getSize() { return GST_BUFFER_SIZE(b); }
uint64_t drivers::gstreamer::Buffer::getTimestamp() { return GST_BUFFER_TIMESTAMP(b); }
uint64_t drivers::gstreamer::Buffer::getDuration() { return GST_BUFFER_DURATION(b); }

void drivers::gstreamer::Buffer::setTimestamp(uint64_t ts)
{
    GST_BUFFER_TIMESTAMP(b) = ts;
}

void drivers::gstreamer::Buffer::setDuration(uint64_t ts)
{
    GST_BUFFER_DURATION(b) = ts;
}

void drivers::gstreamer::Buffer::setTimestampNone() { GST_BUFFER_TIMESTAMP(b) = GST_CLOCK_TIME_NONE; }
void drivers::gstreamer::Buffer::setDurationNone() { GST_BUFFER_DURATION(b) = GST_CLOCK_TIME_NONE; }

drivers::gstreamer::AppSource::AppSource(drivers::gstreamer::Pipeline& p, const std::string& name) : pipe(p)
{
    System::get();
    
    source = gst_bin_get_by_name (GST_BIN (pipe.pipeelem), name.c_str());
    if(source == NULL)
    {
        throw std::runtime_error("Couldn't get SOURCE");
    }
    
    g_signal_connect(source, "need-data", G_CALLBACK(needDataKicker), this);
    g_signal_connect(source, "enough-data", G_CALLBACK(enoughDataKicker), this);
}

drivers::gstreamer::AppSource::~AppSource()
{
    gst_object_unref (GST_OBJECT (source));
}

drivers::gstreamer::Pipeline::State drivers::gstreamer::AppSource::getState()
{
    //uint64_t timeout = GST_CLOCK_TIME_NONE;
    uint64_t timeout = 1000000; // 1 ms
    GstState state; 
    GstStateChangeReturn gscr = gst_element_get_state(GST_ELEMENT (source), &state, NULL, timeout);
    if(gscr != GST_STATE_CHANGE_SUCCESS)
    {
        return Pipeline::State::ERROR;
    }
    else
    {
        return (drivers::gstreamer::Pipeline::State)state;
    }
}

bool drivers::gstreamer::AppSource::pushBuffer(drivers::gstreamer::Buffer& b)
{
    GstFlowReturn ret;
    
    g_signal_emit_by_name(source, "push-buffer", b.b, &ret);
    
    if(ret !=  GST_FLOW_OK)
    {
        return false;
    }
    else
    {
        return true;
    }
}

void drivers::gstreamer::AppSource::setEmitSignals(bool b)
{
    g_object_set(G_OBJECT(source), "emit-signals", b == true ? TRUE : FALSE, NULL);
}

void drivers::gstreamer::AppSource::setMaxBytes(std::size_t b)
{
    g_object_set(G_OBJECT(source), "max-bytes", b, NULL);
}

void drivers::gstreamer::AppSource::setBlock(bool b)
{
    g_object_set(G_OBJECT(source), "block", b == true ? TRUE : FALSE, NULL);
}

void drivers::gstreamer::AppSource::setLive(bool b)
{
    g_object_set(G_OBJECT(source), "is-live", b == true ? TRUE : FALSE, NULL);
}

void drivers::gstreamer::AppSource::setDoTimestamp(bool b)
{
    g_object_set(G_OBJECT(source), "do-timestamp", b == true ? TRUE : FALSE, NULL);
}

void drivers::gstreamer::AppSource::setBlocksize(std::size_t b)
{
    g_object_set(G_OBJECT(source), "blocksize", b, NULL);
}

drivers::gstreamer::AppSink::AppSink(drivers::gstreamer::Pipeline& p, const std::string& name) : pipe(p)
{
    System::get();
    
    sink = gst_bin_get_by_name (GST_BIN (pipe.pipeelem), name.c_str());
    
    if(sink == NULL)
    {
        throw std::runtime_error("Couldn't get SINK");
    }
    
    g_signal_connect(GST_APP_SINK(sink), "new-buffer", G_CALLBACK (newBufferKicker), this);
}

drivers::gstreamer::AppSink::~AppSink()
{
    gst_object_unref (GST_OBJECT (sink));
}

int drivers::gstreamer::AppSink::newBufferKicker(GstElement* sink, void* data)
{
    AppSink* cmp = (AppSink*)data;
    bool ret = cmp->newBuffer();
    if(ret == true)
    {
        return GST_FLOW_OK;
    }
    else
    {
        return GST_FLOW_ERROR;
    }
}

drivers::gstreamer::Pipeline::State drivers::gstreamer::AppSink::getState()
{
    //uint64_t timeout = GST_CLOCK_TIME_NONE;
    uint64_t timeout = 1000000; // 1 ms
    GstState state; 
    GstStateChangeReturn gscr = gst_element_get_state(GST_ELEMENT (sink), &state, NULL, timeout);
    if(gscr != GST_STATE_CHANGE_SUCCESS)
    {
        return Pipeline::State::ERROR;
    }
    else
    {
        return (drivers::gstreamer::Pipeline::State)state;
    }
}

bool drivers::gstreamer::AppSink::pullBuffer(drivers::gstreamer::Buffer& b)
{
    g_signal_emit_by_name (sink, "pull-buffer", &(b.b));
    if(b.b) 
    {   
        return true;
    }
    else
    {
        return false;
    }
}

void drivers::gstreamer::AppSink::setEmitSignals(bool b)
{
    g_object_set(G_OBJECT(sink), "emit-signals", b == true ? TRUE : FALSE, NULL);
}

void drivers::gstreamer::AppSink::setSync(bool b)
{
    g_object_set(G_OBJECT(sink), "sync", b == true ? TRUE : FALSE, NULL);
}

void drivers::gstreamer::AppSink::setDrop(bool b)
{
    g_object_set(G_OBJECT(sink), "drop", b == true ? TRUE : FALSE, NULL);
}




