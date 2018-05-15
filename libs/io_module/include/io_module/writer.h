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
        Writer(const std::string config_file_loc);

    private:
        // Implement methods for writing to YARP ports.
        virtual void SetConfigs(const std::string config_file_loc);

        virtual void SetDrivers();

        virtual void SetPorts();
};

#endif