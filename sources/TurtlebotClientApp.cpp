#include <iostream>
#include <chrono>
#include <atomic>
#include <thread>
#include <signal.h>

#include <misccpp/nanomsg.hpp>

#include <misc_drivers/Joystick.hpp>
#include <misc_drivers/TurtlebotClientServer.hpp>

static constexpr float joyReducer = 0.75f;

std::atomic<bool> do_exit;
static const int StopSignal = SIGINT;

static void catcher( int sig )
{
    if(sig != StopSignal)
    {   
        abort();
    }
}

void joystickPuller(drivers::robot::TurtlebotClient& robot, drivers::sensor::Joystick& joy)
{
    const float ascale = 0.9f;
    const float lscale = 0.3f;
    float axis_angular = 0.0f, axis_linear = 0.0f;
    bool btn_fire = false;
    bool motion_disabled = true;
    
    std::chrono::high_resolution_clock::time_point last_pub = std::chrono::high_resolution_clock::now();
    
    while(!do_exit)
    {
        drivers::sensor::Joystick::JoystickEvent ev;
        
        if(joy.read(ev))
        {
            if(ev.Type == drivers::sensor::Joystick::JoystickEvent::Button)
            {
                if(ev.Index == 0)
                {
                    bool new_btn_fire = ev.Value == 0 ? false : true;
                    if(btn_fire == true && new_btn_fire == false) 
                    { 
                        motion_disabled = true; 
                        
                        try
                        {
                            robot.motionStop();
                        }
                        catch(const std::exception& ex)
                        {
                            
                        }
                    }
                    if(btn_fire == false && new_btn_fire == true) { motion_disabled = false; }
                    btn_fire = new_btn_fire;
                }
            }
            else if(ev.Type == drivers::sensor::Joystick::JoystickEvent::Axis)
            {
                if(ev.Index == 0)
                {
                    axis_angular = -(float)ev.Value / 32768.0f;
                }
                else if(ev.Index == 1)
                {
                    axis_linear = -(float)ev.Value / 32768.0f;
                }
            }
            
            const float vel_angular = joyReducer * ascale *  axis_angular;
            const float vel_linear = joyReducer * lscale * axis_linear;
            
            if(!motion_disabled)
            {
                std::chrono::high_resolution_clock::time_point currt = std::chrono::high_resolution_clock::now();
                int since_last = std::chrono::duration_cast<std::chrono::milliseconds>(currt - last_pub).count();
                if(since_last > 30)
                {
                    try
                    {
                        robot.motionTwist(vel_linear, vel_angular);
                    }
                    catch(const std::exception& ex)
                    {
                        
                    }
                    
                    last_pub = currt;
                }
            }
            
        }
    }
}


int main(int argc, char** argv)
{
    std::string joystick_path = "/dev/input/js0";
    std::string hostname = "localhost";
    
    if(argc == 2)
    {
        hostname = std::string(argv[1]);
    }
    else if(argc == 3)
    {
        hostname = std::string(argv[1]);
        joystick_path = std::string(argv[2]);
    }
    
    std::cout << "Turtlebot Client: Use: " << argv[0] << " [HOSTNAME] [JOYSTICK_DEV]" << std::endl;
    std::cout << "Turtlebot Client: Running with: " << argv[0] << " " << hostname << " " << joystick_path << std::endl;
    
    do_exit = false;
    
    drivers::sensor::Joystick js(joystick_path);
    
    if(!js.isConnected())
    {
        std::cerr << "Turtlebot Client: Cannot open joystick device " << joystick_path << std::endl;
        return 1;
    }
    
    drivers::robot::TurtlebotClient robot;
    
    robot.connect(hostname);
    
    if(!robot.isConnected())
    {
        std::cerr << "Turtlebot Client: Cannot connect to " << hostname << std::endl;
        return 1;
    }
    else
    {
        std::cout << "Turtlebot Client: Got joystick " << js.getName() << " / " << js.getAxisCount() << " / " << js.getButtonCount() << std::endl;
    }
    
    std::thread thr_joy(joystickPuller, std::ref(robot), std::ref(js));

    std::cout << "Turtlebot Client: Running, press Ctrl-C to stop" << std::endl;
    
    // wait for StopSignal
    {   
        struct sigaction sigact;
        sigset_t waitset;
        int sig;
        int result = 0;
        
        sigemptyset(&sigact.sa_mask);
        sigact.sa_flags = 0;
        sigact.sa_handler = catcher;
        sigaction(StopSignal, &sigact, NULL);
        
        sigemptyset(&waitset );
        sigaddset(&waitset, StopSignal);
        
        result = sigwait(&waitset, &sig);
        if(result == 0)
        {
            if((sig != StopSignal))
            {
                abort();
            }
        }
    }
    
    do_exit = true;
    thr_joy.join();

    robot.disconnect();
    
    std::cout << "Turtlebot Client: Done" << std::endl;
    
    nn::terminate();
    
    return 0;
}
