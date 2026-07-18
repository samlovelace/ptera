#include <iostream> 
#include "Colors.hpp"
#include "InputHandler.h"
#include "CommanderComms.h"
#include <signal.h>


void signaled(int)
{
    exit(1); 
}

int main()
{
    signal(SIGINT, signaled); 
//     std::cout << R"(
//      ___      .______      .___  ___.      ______   ______   .___  ___. .___  ___.      ___      .__   __.  _______   _______ .______      
//     /   \     |   _  \     |   \/   |     /      | /  __  \  |   \/   | |   \/   |     /   \     |  \ |  | |       \ |   ____||   _  \     
//    /  ^  \    |  |_)  |    |  \  /  |    |  ,----'|  |  |  | |  \  /  | |  \  /  |    /  ^  \    |   \|  | |  .--.  ||  |__   |  |_)  |    
//   /  /_\  \   |      /     |  |\/|  |    |  |     |  |  |  | |  |\/|  | |  |\/|  |   /  /_\  \   |  . `  | |  |  |  ||   __|  |      /     
//  /  _____  \  |  |\  \----.|  |  |  |    |  `----.|  `--'  | |  |  |  | |  |  |  |  /  _____  \  |  |\   | |  '--'  ||  |____ |  |\  \----.
// /__/     \__\ | _| `._____||__|  |__|     \______| \______/  |__|  |__| |__|  |__| /__/     \__\ |__| \__| |_______/ |_______|| _| `._____|
//     )" << std::endl;

    InputHandler inputHandler; 
    
    CommanderComms comms; 
    comms.start(); 

    std::string command; 

    while(true)
    {
        std::cout << GREEN << "Command: "; 
        std::getline(std::cin, command); 

        if("exit" == command)
        {
            std::cout << "Exiting..." << std::endl; 
            break; 
        }

        inputHandler.handle(command); 
    }

    comms.stop(); 
}
