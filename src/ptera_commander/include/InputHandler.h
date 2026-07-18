#ifndef INPUTHANDLER_H
#define INPUTHANDLER_H

#include "Colors.hpp"
#include <string> 
 
class InputHandler 
{ 
public:
    InputHandler();
    ~InputHandler();

    void handle(const std::string& anInput); 

private:

    std::string mPackagePath; 
   
};
#endif //INPUTHANDLER_H