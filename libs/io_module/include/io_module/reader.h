#ifndef IO_MODULE_READ_H_
#define IO_MODULE_READ_H_

#include <stdio.h>

#include <yarp/os/RateThread.h>
#include <yarp/os/Network.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include <yarp/os/Time.h>
#include <yarp/eigen/Eigen.h>
#include <ncurses.h>

#include "network_manager.h"
#include "utils.h"

// Wrapper class for YARP to read from ports.
//
// Implemented by Martin Huber.
class Reader
{
    public:
        
        // Constructor. Set up the network.
        Reader();

    private:

        // Methods to implement for reading from YARP ports.
        virtual void SetConfigs() = 0;

        virtual void SetDrivers() = 0;

        virtual void SetPorts() = 0;
};


// KeyReader implements a simple user interface that
// supports the user with w, a, s, d controls to write
// to a port via WriteToPort().
//
// Implemented by Martin Huber.
class KeyReader
{
    public:
        KeyReader();

        ~KeyReader();

    private:

        // Port for sending velocities.
        yarp::os::BufferedPort<yarp::sig::Vector> port;

        // Read incomming commands and update the velocity.
        void ReadCommands();
    
        // Set velocity.
        void SetVelocity(Eigen::Vector3d& acc, double t);

        // Write to port.
        void WriteToPort();

        // Accelerations.
        Eigen::Vector3d acc_w_, acc_a_, acc_s_, acc_d_;

        // Respective time, accelerated in one direction.
        double t_iter_;

        // Velocity.
        yarp::sig::Vector vel_;

        // User interface.
        WINDOW *win_w_, *win_a_, *win_s_, *win_d_;
        WINDOW *win_q_, *win_e_, *win_info_, *win_vel_;
};

#endif