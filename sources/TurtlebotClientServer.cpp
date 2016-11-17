#include <sstream>
#include <atomic>
#include <cmath>
#include <chrono>

#include <nanomsg.hpp>

#include <cereal/cereal.hpp>
#include <cereal/types/string.hpp>
#include <cereal/types/vector.hpp>
#include <cereal/types/map.hpp>
#include <cereal_raw_binary.hpp>

#include <TurtlebotDriver.hpp>

#include <TurtlebotClientServer.hpp>

template<typename T>
static inline T constrainAngle(T x)
{
    x = fmod(x, T(2.0*M_PI) );
    if (x < T(0.0)) { x += T(2.0*M_PI); }
    return x;
}

template<class Archive>
void drivers::robot::TurtlebotState::serialize(Archive& archive)
{
    archive(CEREAL_NVP(TransmittedTimestamp));
    archive(CEREAL_NVP(ReceivedTimestamp));
    archive(CEREAL_NVP(InternalTimestamp));
    archive(CEREAL_NVP(PositionX));
    archive(CEREAL_NVP(PositionY));
    archive(CEREAL_NVP(PositionTheta));
    archive(CEREAL_NVP(VelocityLeft));
    archive(CEREAL_NVP(VelocityRight));
    archive(CEREAL_NVP(BumperRight));
    archive(CEREAL_NVP(BumperCentral));
    archive(CEREAL_NVP(BumperLeft));
    archive(CEREAL_NVP(WheelDropRight));
    archive(CEREAL_NVP(WheelDropLeft));
    archive(CEREAL_NVP(CliffRight));
    archive(CEREAL_NVP(CliffCentral));
    archive(CEREAL_NVP(CliffLeft));
    archive(CEREAL_NVP(BatteryVoltage));
    archive(CEREAL_NVP(OvercurrentLeft));
    archive(CEREAL_NVP(OvercurrentRight));
}

struct TurtlebotCommand
{
    enum CmdType
    {
        KeepAlive = 0,
        MotionStop,
        MotionRotate,
        MotionTranslate,
        MotionTwist,
        ResetOdometry
    };
    
    CmdType Type;
    float   Field1;
    float   Field2;
    
    TurtlebotCommand() : Type(KeepAlive), Field1(0.0f), Field2(0.0f) { }
    TurtlebotCommand(CmdType t, float f1 = 0.0f, float f2 = 0.0f) : Type(t), Field1(f1), Field2(f2) { }
    
    template<class Archive>
    void serialize(Archive& archive)
    {
        archive(CEREAL_NVP(Type));
        archive(CEREAL_NVP(Field1));
        archive(CEREAL_NVP(Field2));
    }
};

struct drivers::robot::TurtlebotServer::TurtlebotServerPimpl
{
    TurtlebotServerPimpl(const std::string& ttydev)  
        : robotPort(ttydev), sock_pub(NN_PUB), 
          sock_ctrl(NN_PAIR), 
          ep_pub(-1), 
          ep_ctrl(-1), 
          do_exit(false),
          got_first(false),
          odometry_x(0.0f), odometry_y(0.0f), odometry_theta(0.0f)
    {
    }
    
    ~TurtlebotServerPimpl()
    {
        stop();
    }
    
    void start(unsigned int ctrl_port = 5577, unsigned int pub_port = 5588)
    {
        if(isStarted()) { stop(); }
        
        std::stringstream ss;
        
        ss << "tcp://*:" << ctrl_port;
        
        ep_ctrl = sock_ctrl.bind(ss.str().c_str());
        sock_ctrl.setsockopt<int>(NN_SOL_SOCKET, NN_RCVMAXSIZE, -1); // NOTE: No limit
        sock_ctrl.setsockopt<int>(NN_SOL_SOCKET, NN_RCVBUF, sizeof(TurtlebotCommand) * 5);
        
        ss.str("");
        
        sock_ctrl.setsockopt<int>(NN_SOL_SOCKET, NN_SNDTIMEO, 1000);
        sock_ctrl.setsockopt<int>(NN_SOL_SOCKET, NN_RCVTIMEO, 1000);
        
        ss << "tcp://*:" << pub_port;
        
        ep_pub = sock_pub.bind(ss.str().c_str());
        sock_pub.setsockopt<int>(NN_SOL_SOCKET, NN_SNDBUF, sizeof(TurtlebotState) * 50);
        
        robot.reset(new drivers::robot::Turtlebot(robotPort, 115200));
        robot->HandleBasicSensorData = std::bind(&TurtlebotServerPimpl::reporter, this, std::placeholders::_1);
        
        do_exit = false;
        thr_control = std::thread(&TurtlebotServerPimpl::controller, this);
    }
    
    bool isStarted() const
    {
        return ep_ctrl >= 0;
    }
    
    void stop()
    {
        if(isStarted())
        {
            do_exit = true;
            thr_control.join();
            robot.reset();
            robot->HandleBasicSensorData = std::function<void (const drivers::robot::Turtlebot::BasicSensorData&)>();
            sock_ctrl.shutdown(ep_ctrl);
            sock_pub.shutdown(ep_pub);
            ep_ctrl = ep_pub = -1;
        }
    }
    
    /**
     * Receive from the robot and publish.
     */
    void reporter(const drivers::robot::Turtlebot::BasicSensorData& bsd)
    {
        if(got_first == false)
        {
            prev_bsd = bsd;
            got_first = true;
            return;
        }

        TurtlebotState state_out;
        
        float dt = (float)((int16_t)((bsd.Timestamp - prev_bsd.Timestamp) & 0xffff)) / 1000.0f; // s
        
        if(dt < 0.001f)
        {   
            return; // dubious
        }
        
        // encoder ticks
        const int16_t dencl = (int16_t)((bsd.EncoderLeft - prev_bsd.EncoderLeft) & 0xffff); // tics
        const int16_t dencr = (int16_t)((bsd.EncoderRight - prev_bsd.EncoderRight) & 0xffff); // tics
        prev_bsd = bsd;
        
        // velocities
        state_out.VelocityLeft = (((float)dencl) * drivers::robot::Turtlebot::TickToMeter) / dt;
        state_out.VelocityRight = (((float)dencr) * drivers::robot::Turtlebot::TickToMeter) / dt;
        
        if((fabs(state_out.VelocityLeft) > 0.0001f) || (fabs(state_out.VelocityRight) > 0.0001f)) // must have motion
        {
            const float trav_l = (drivers::robot::Turtlebot::TickToMeter * (float)dencl);
            const float trav_r = (drivers::robot::Turtlebot::TickToMeter * (float)dencr);
            
            const float dx = (trav_l + trav_r) / 2.0f;
            
            const float dtheta = (trav_r - trav_l) / drivers::robot::Turtlebot::RobotWheelBase;
            
            odometry_theta = constrainAngle(odometry_theta + dtheta);
            
            odometry_x = odometry_x + dx * cos(odometry_theta);
            odometry_y = odometry_y + dx * sin(odometry_theta);
        }     
        
        state_out.PositionX = odometry_x;
        state_out.PositionY = odometry_y;
        state_out.PositionTheta = odometry_theta;
        
        state_out.InternalTimestamp = bsd.Timestamp;
        state_out.ReceivedTimestamp = 0;
        state_out.BumperRight = bsd.BumperRight;
        state_out.BumperCentral = bsd.BumperCentral;
        state_out.BumperLeft = bsd.BumperLeft;
        state_out.WheelDropRight = bsd.WheelDropRight;
        state_out.WheelDropLeft = bsd.WheelDropLeft;
        state_out.CliffRight = bsd.CliffRight;
        state_out.CliffCentral = bsd.CliffCentral;
        state_out.CliffLeft = bsd.CliffLeft;
        state_out.BatteryVoltage = bsd.BatteryVoltage;
        state_out.WheelDropLeft = bsd.WheelDropLeft;
        state_out.OvercurrentLeft = bsd.OvercurrentRight;
        
        std::chrono::high_resolution_clock::time_point tp = std::chrono::high_resolution_clock::now();
        std::chrono::nanoseconds d = tp.time_since_epoch();
        
        state_out.TransmittedTimestamp = d.count();
        
        cereal::RawByteCountingArchive rbca;
        
        rbca(state_out);
        
        nn::message_t msgout(rbca.totalBytes());
        cereal::RawBinaryOutputArchive rboa(msgout.data(), msgout.size());
        
        rboa(state_out);
        
        try
        {   
            sock_pub.send(msgout);
        }
        catch(const nn::exception& ex)
        {   
            throw std::runtime_error("State sending error");
        }
    }
    
    /**
     * Receive commands form the client and handle.
     */
    void controller()
    {
        while(!do_exit)
        {
            try
            {   
                // receive message
                nn::message_t msgin;
                sock_ctrl.receive(msgin);
                
                cereal::RawBinaryInputArchive rbia(msgin.data(), msgin.size());
                
                TurtlebotCommand cmd;
                rbia(cmd);
                
                if(robot)
                {
                    switch(cmd.Type)
                    {
                        case TurtlebotCommand::KeepAlive:
                            robot->requestFirmwareVersion(); // just any command
                            break;
                        case TurtlebotCommand::MotionStop:
                            robot->motionStop();
                            break;
                        case TurtlebotCommand::MotionRotate:
                            robot->motionRotate(cmd.Field1);
                            break;
                        case TurtlebotCommand::MotionTranslate:
                            robot->motionTranslate(cmd.Field1);
                            break;
                        case TurtlebotCommand::MotionTwist:
                            robot->motionTwist(cmd.Field1, cmd.Field2);
                            break;
                        case TurtlebotCommand::ResetOdometry:
                            resetOdometry();
                            break;
                    }
                }
            }
            catch(const nn::exception& ex)
            {   
                // what to do?
            }
            catch(const std::exception& ex)
            {   
                // what to do?
            }
            catch(...)
            {
                // what to do?
            }
        }
    }
    
    void resetOdometry()
    {
        odometry_x = odometry_y = odometry_theta = 0.0f;
    }
    
    std::string robotPort;
    nn::socket_t sock_pub;
    nn::socket_t sock_ctrl;
    nn::endpoint_id ep_pub;
    nn::endpoint_id ep_ctrl;
    std::atomic<bool> do_exit;
    std::unique_ptr<drivers::robot::Turtlebot> robot;
    std::thread thr_control;
    drivers::robot::Turtlebot::BasicSensorData prev_bsd;
    bool got_first;
    float odometry_x, odometry_y, odometry_theta;
};

// thin redirection to the Pimpl
drivers::robot::TurtlebotServer::TurtlebotServer(const std::string& ttydev) : pimpl(new TurtlebotServerPimpl(ttydev)) { }
drivers::robot::TurtlebotServer::~TurtlebotServer() { }
void drivers::robot::TurtlebotServer::start(unsigned int ctrl_port, unsigned int pub_port) { pimpl->start(ctrl_port, pub_port); }
bool drivers::robot::TurtlebotServer::isStarted() const { return pimpl->isStarted(); }
void drivers::robot::TurtlebotServer::stop() { pimpl->stop(); }

struct drivers::robot::TurtlebotStateListener::TurtlebotStateListenerPimpl
{
    TurtlebotStateListenerPimpl() : sock_sub(NN_SUB), ep_sub(-1)
    {
        
    }
    
    ~TurtlebotStateListenerPimpl()
    {
        disconnect();
    }
    
    void connect(const std::string& ipaddr, unsigned int sub_port = 5588)
    {
        if(isConnected()) { disconnect(); }
        
        std::stringstream ss;
        
        ss << "tcp://" << ipaddr << ":" << sub_port;
        
        ep_sub = sock_sub.connect(ss.str().c_str());
        sock_sub.setsockopt(NN_SUB, NN_SUB_SUBSCRIBE, "", 0);
        sock_sub.setsockopt<int>(NN_SOL_SOCKET, NN_RCVTIMEO, 1000);
        sock_sub.setsockopt<int>(NN_SOL_SOCKET, NN_RCVMAXSIZE, -1); // NOTE: No limit
        sock_sub.setsockopt<int>(NN_SOL_SOCKET, NN_RCVBUF, sizeof(TurtlebotState) * 50);
        
        do_exit = false;
        thr_sub = std::thread(&TurtlebotStateListenerPimpl::stateReceiver, this);
    }
    
    bool isConnected() const
    {
        return ep_sub >= 0;
    }
    
    void disconnect()
    {
        if(isConnected())
        {
            do_exit = true;
            thr_sub.join();
            sock_sub.shutdown(ep_sub);
            ep_sub = -1;
        }
    }
    
    void setStateHandler(StateHandlerT sh)
    {
        csh = sh;
    }
    
    void stateReceiver()
    {
        while(!do_exit)
        {
            try
            {   
                // receive message
                nn::message_t msgin;
                sock_sub.receive(msgin);
                
                std::chrono::high_resolution_clock::time_point tp = std::chrono::high_resolution_clock::now();
                std::chrono::nanoseconds d = tp.time_since_epoch();
                
                cereal::RawBinaryInputArchive rbia(msgin.data(), msgin.size());
                
                TurtlebotState state;
                rbia(state);
                state.ReceivedTimestamp = d.count();
                if(csh)
                {
                    // run state handler
                    csh(state);
                }
            }
            catch(const nn::exception& ex)
            {   
                // FIXME what to do?
            }
            catch(const std::exception& ex)
            {   
                // FIXME what to do?
            }
            catch(...)
            {
                // FIXME what to do?
            }
        }
    }
    
    nn::socket_t sock_sub;
    nn::endpoint_id ep_sub;
    std::atomic<bool> do_exit;
    std::thread thr_sub;
    StateHandlerT csh;
};

// thin redirection to the Pimpl
drivers::robot::TurtlebotStateListener::TurtlebotStateListener(): pimpl(new TurtlebotStateListenerPimpl()) { }
drivers::robot::TurtlebotStateListener::~TurtlebotStateListener() { }
void drivers::robot::TurtlebotStateListener::connect(const std::string& ipaddr, unsigned int sub_port) { pimpl->connect(ipaddr, sub_port); }
bool drivers::robot::TurtlebotStateListener::isConnected() const { return pimpl->isConnected(); }
void drivers::robot::TurtlebotStateListener::disconnect() { pimpl->disconnect(); }
void drivers::robot::TurtlebotStateListener::setStateHandler(typename TurtlebotStateListener::StateHandlerT sh) { pimpl->setStateHandler(sh); }

struct drivers::robot::TurtlebotClient::TurtlebotClientPimpl
{
    TurtlebotClientPimpl(unsigned int watchdog_timeout)
        : sock_ctrl(NN_PAIR), 
          ep_ctrl(-1), 
          do_exit(false),
          wdt(watchdog_timeout)
        
    {
        
    }
    
    ~TurtlebotClientPimpl()
    {
        disconnect();
    }
    
    void connect(const std::string& ipaddr, unsigned int ctrl_port, unsigned int sub_port)
    {
        if(isConnected()) { disconnect(); }
        
        std::stringstream ss;
        
        ss << "tcp://" << ipaddr << ":" << ctrl_port;
        
        ep_ctrl = sock_ctrl.connect(ss.str().c_str());
        ss.str("");
        
        sock_ctrl.setsockopt<int>(NN_SOL_SOCKET, NN_SNDTIMEO, 1000);
        sock_ctrl.setsockopt<int>(NN_SOL_SOCKET, NN_RCVTIMEO, 1000);
        sock_ctrl.setsockopt<int>(NN_SOL_SOCKET, NN_SNDBUF, sizeof(TurtlebotCommand) * 5);
        
        do_exit = false;
        thr_wdt = std::thread(&TurtlebotClientPimpl::thread_watchdog, this);
        
        tsl.connect(ipaddr, sub_port);
    }
    
    bool isConnected() const
    {
        return (ep_ctrl >= 0) && (tsl.isConnected());
    }
    
    void disconnect()
    {
        if(isConnected())
        {
            do_exit = true;
            thr_wdt.join();
            sock_ctrl.shutdown(ep_ctrl);
            ep_ctrl = -1;
            tsl.disconnect();
        }
    }
    
    void motionStop()
    {
        sendCommand(TurtlebotCommand(TurtlebotCommand::MotionStop));
    }
    
    void motionRotate(float rv)
    {
        sendCommand(TurtlebotCommand(TurtlebotCommand::MotionRotate,rv));
    }
    
    void motionTranslate(float speed)
    {
        sendCommand(TurtlebotCommand(TurtlebotCommand::MotionTranslate,speed));
    }
    
    void motionTwist(float linear, float angular)
    {
        sendCommand(TurtlebotCommand(TurtlebotCommand::MotionTwist,linear,angular));
    }
    
    void keepAlive()
    {
        sendCommand(TurtlebotCommand(TurtlebotCommand::KeepAlive));
    }
    
    void resetOdometry()
    {
        sendCommand(TurtlebotCommand(TurtlebotCommand::ResetOdometry));
    }
    
    void sendCommand(const TurtlebotCommand& cmd)
    {
        cereal::RawByteCountingArchive rbca;
        
        rbca(cmd);
        
        nn::message_t msgout(rbca.totalBytes());
        cereal::RawBinaryOutputArchive rboa(msgout.data(), msgout.size());
        
        rboa(cmd);
        
        try
        {   
            sock_ctrl.send(msgout);
        }
        catch(const nn::exception& ex)
        {   
            throw std::runtime_error("Command sending error");
        }
    }
    
    void thread_watchdog()
    {
        while(!do_exit)
        {
            if(isConnected())
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(wdt));
                keepAlive();
                // TODO maybe check if motion active
            }
        }
    }
    
    nn::socket_t sock_ctrl;
    nn::endpoint_id ep_ctrl;
    std::atomic<bool> do_exit;
    std::thread thr_wdt;
    unsigned int wdt;
    TurtlebotStateListener tsl;
};

// thin redirection to the Pimpl
drivers::robot::TurtlebotClient::TurtlebotClient(unsigned int watchdog_timeout): pimpl(new TurtlebotClientPimpl(watchdog_timeout)) { }
drivers::robot::TurtlebotClient::~TurtlebotClient() { }
void drivers::robot::TurtlebotClient::connect(const std::string& ipaddr, unsigned int ctrl_port, unsigned int sub_port) { pimpl->connect(ipaddr, ctrl_port, sub_port); }
bool drivers::robot::TurtlebotClient::isConnected() const { return pimpl->isConnected(); }
void drivers::robot::TurtlebotClient::disconnect() { pimpl->disconnect(); }
void drivers::robot::TurtlebotClient::setStateHandler(typename TurtlebotStateListener::StateHandlerT sh) { pimpl->tsl.setStateHandler(sh); }
void drivers::robot::TurtlebotClient::motionStop() { pimpl->motionStop(); }
void drivers::robot::TurtlebotClient::motionRotate(float rv) { pimpl->motionRotate(rv); }
void drivers::robot::TurtlebotClient::motionTranslate(float speed) { pimpl->motionTranslate(speed); }
void drivers::robot::TurtlebotClient::motionTwist(float linear, float angular) { pimpl->motionTwist(linear,angular); }
void drivers::robot::TurtlebotClient::keepAlive() { pimpl->keepAlive(); }
void drivers::robot::TurtlebotClient::resetOdometry() { pimpl->resetOdometry(); }
