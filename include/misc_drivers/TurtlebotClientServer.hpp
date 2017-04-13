#ifndef TURTLEBOT_CLIENT_SERVER_HPP
#define TURTLEBOT_CLIENT_SERVER_HPP

#include <cstdint>
#include <string>
#include <memory>
#include <functional>

namespace drivers
{
namespace robot
{

struct TurtlebotState
{
    uint64_t        TransmittedTimestamp;
    uint64_t        ReceivedTimestamp;
    uint16_t        InternalTimestamp;
    float           PositionX, PositionY, PositionTheta;
    float           VelocityLeft, VelocityRight;
    bool            BumperRight, BumperCentral, BumperLeft;
    bool            WheelDropRight, WheelDropLeft;
    bool            CliffRight, CliffCentral, CliffLeft;
    float           BatteryVoltage;
    bool            OvercurrentLeft, OvercurrentRight;
    
    template<class Archive>
    void serialize(Archive& archive);
};

class TurtlebotServer
{
    struct TurtlebotServerPimpl;
public:    
    TurtlebotServer(const std::string& ttydev);
    virtual ~TurtlebotServer();
    
    void start(unsigned int ctrl_port = 5577, unsigned int pub_port = 5588);
    bool isStarted() const;
    void stop();
    
private:
    std::unique_ptr<TurtlebotServerPimpl> pimpl;
};

class TurtlebotStateListener
{
    struct TurtlebotStateListenerPimpl;
public:
    typedef std::function<void (const TurtlebotState&)> StateHandlerT;
    
    TurtlebotStateListener();
    virtual ~TurtlebotStateListener();
    
    void connect(const std::string& ipaddr, unsigned int sub_port = 5588);
    bool isConnected() const;
    void disconnect();
    
    void setStateHandler(StateHandlerT sh);
private:
    std::unique_ptr<TurtlebotStateListenerPimpl> pimpl;
};

class TurtlebotClient
{
    struct TurtlebotClientPimpl;
public:
    TurtlebotClient(unsigned int watchdog_timeout = 1000);
    virtual ~TurtlebotClient();
    
    void connect(const std::string& ipaddr, unsigned int ctrl_port = 5577, unsigned int sub_port = 5588);
    bool isConnected() const;
    void disconnect();
    
    void setStateHandler(typename TurtlebotStateListener::StateHandlerT sh);
    
    void motionStop();
    void motionRotate(float rv);
    void motionTranslate(float speed);
    void motionTwist(float linear, float angular);
    void keepAlive();
    void resetOdometry();
private:
    std::unique_ptr<TurtlebotClientPimpl> pimpl;
};
    
}
}

#endif // TURTLEBOT_CLIENT_SERVER_HPP
