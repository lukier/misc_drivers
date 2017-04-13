#include <iostream>
#include <signal.h>

#include <misccpp/nanomsg.hpp>

#include <misc_drivers/TurtlebotClientServer.hpp>

static const int StopSignal = SIGINT;

static void catcher( int sig )
{
    if(sig != StopSignal)
    {   
        abort();
    }
}

int main(int argc, char** argv)
{
    std::string port = "/dev/ttyUSB0";
    
    if(argc == 2)
    {
        port = std::string(argv[1]);
    }
    
    std::cout << "Turtlebot Server: Use: " << argv[0] << " [SERIAL_DEV]" << std::endl;
    std::cout << "Turtlebot Server: Running with: " << argv[0] << " " << port << std::endl;
    
    try
    {
        drivers::robot::TurtlebotServer srv(port);
    
        srv.start();
    
        std::cout << "Turtlebot Server: Running, press Ctrl-C to stop" << std::endl;
        
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
        
        srv.stop();
        
        std::cout << "Turtlebot Server: Done" << std::endl;
    }
    catch(const std::exception& ex)
    {
        std::cerr << "Turtlebot Server: Cannot connect to the robot" << std::endl;
        return 1;
    }
    
    nn::terminate();
    
    return 0;
}
