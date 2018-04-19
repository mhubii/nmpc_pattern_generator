#ifndef IO_MODULE_READ_H_
#define IO_MODULE_READ_H_

#include <stdio.h>

#include <yarp/os/RateThread.h>
#include <yarp/os/Network.h>

#include "network_manager.h"
#include "utils.h"

// Wrapper class for YARP to read from ports.
//
// Implemented by Martin Huber.
class Reader
{
    public:
        // Constructor. Set up the network.
        Reader() : yarp() {  };

    private:
        // YARP network.
        yarp::os::Network yarp;

        // Methods to implement for reading from YARP ports.
        virtual void SetConfigs() = 0;

        virtual void SetDrivers() = 0;

        virtual void SetPorts() = 0;
};

class KeyReader : public yarp::os::RateThread, public Reader
{
    public:
        KeyReader(const double period) : yarp::os::RateThread(period), Reader::Reader() {  };

    private:
        // Implement methods for RateThread.
        virtual void run() {  };

        // Implement methods for reading from YARP ports.
        virtual void SetConfigs() {};

        virtual void SetDrivers() {};

        virtual void SetPorts() {};

        // Linear and angular velocities.


};

class AppReader : public yarp::os::RateThread
{

};

#endif