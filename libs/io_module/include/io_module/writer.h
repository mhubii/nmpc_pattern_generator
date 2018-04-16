#ifndef IO_MODULE_WRITE_H_
#define IO_MODULE_WRITE_H_

#include <yarp/os/RateThread.h>

#include "utils.h"

// Wrapper class for YARP to write to ports.
//
// Implemented by Martin Huber
class Writer
{
    public:
        Writer();

        void WriteToJoints();
};

#endif