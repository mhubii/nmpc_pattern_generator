#ifndef IO_MODULE_READ_H_
#define IO_MODULE_READ_H_

#include "utils.h"

// Wrapper class for YARP to read from ports.
//
// Implemented by Martin Huber.
class Reader
{
    public:
        Reader();

        void ReadFromCamera();

        void ReadFromJoints();
};

#endif