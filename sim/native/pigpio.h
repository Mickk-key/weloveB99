#pragma once
#include <stdexcept>
#define PI_OUTPUT 1
inline int gpioInitialise(){throw std::runtime_error("Simulation build has NO GPIO");}
inline void gpioTerminate(){}
inline int gpioPWM(unsigned,unsigned){throw std::runtime_error("Simulation build has NO GPIO");}
inline int gpioSetMode(unsigned,unsigned){return 0;}
inline int gpioSetPWMfrequency(unsigned,unsigned f){return f;}
inline int gpioSetPWMrange(unsigned,unsigned){return 0;}
