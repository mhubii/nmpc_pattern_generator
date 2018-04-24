#ifndef IO_MODULE_READ_H_
#define IO_MODULE_READ_H_

#include <stdio.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Network.h>
<<<<<<< HEAD
#include <Eigen/Dense>
=======
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include <yarp/os/Time.h>
#include <yarp/eigen/Eigen.h>
#include <ncurses.h>
>>>>>>> 9efb23a88150b355067ba3223d22360128c4404e

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
<<<<<<< HEAD
#include <iostream>
// KeyCommands is an implementation of the YARP RFModule
// for controlling the robots velocity in the terminal
// via the keyboard. Possible commands are listed in the
// command enum. Precisely, the commands represent accelerations
// that are in turn converted into velocities.
//
// Implemented by Martin Huber.
class KeyCommands : public yarp::os::RFModule
{
    public:
        // Port to handle messages.
        yarp::os::Port port;

        // Velocity.
        Eigen::Vector3d vel_;

        // Keyboard commands.
        enum command : char {UP = 'w', DOWN = 's', LEFT = 'a', RIGHT = 'd'};

        // Determines the vel_, given the command.
        void SetVelocity(command in) {};

        // Methods implemented for the RFModule.
        double getPeriod() { return 100; };

        bool updateModule() { std::cout << "update" << std::endl; };

        bool respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply) {
            if (command.get(0).asString() == "quit")
                return false;
            else if (command.get(0).asVocab() == UP){
                std::cout << "Got up." << std::endl;}
            else if (command.get(0).asVocab() == DOWN){
                std::cout << "Got down." << std::endl;}
            else if (command.get(0).asVocab() == LEFT){
                std::cout << "Got left." << std::endl;}
            else if (command.get(0).asVocab() == RIGHT){
                std::cout << "Got right." << std::endl;}

            return true;
        };

        bool configure(yarp::os::ResourceFinder& rf) { 
            port.open("/in");
            attach(port);
            return true; 
        };

        bool close() { 
            port.close();
            return true; 
        };
};

// AppCommands is an implementation of the YARP RFModule
// for controlling the robots velocity via an app.
// Possible commands are listed in the command enum.
//
// Implemented by Martin Huber.
// class AppCommands : public yarp::os::RFModule
// {

// };

=======


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
>>>>>>> 9efb23a88150b355067ba3223d22360128c4404e

        // Accelerations.
        Eigen::Vector3d acc_w_, acc_a_, acc_s_, acc_d_;

<<<<<<< HEAD
using namespace std;
using namespace yarp::os;

class MyModule:public RFModule
{
    Port handlerPort; // a port to handle messages
    int count;
public:
    double getPeriod()
    {
        // module periodicity (seconds), called implicitly by the module.
        return 1.0;
    }
    // This is our main function. Will be called periodically every getPeriod() seconds
    bool updateModule()
    {
        count++;
        cout << "[" << count << "]" << " updateModule..." << endl;
        return true;
    }
    // Message handler. Just echo all received messages.
    bool respond(const Bottle& command, Bottle& reply)
    {
        cout << "Got something, echo is on" << endl;
        if (command.get(0).asString() == "quit")
            return false;
        else
            reply = command;
        return true;
    }
    // Configure function. Receive a previously initialized
    // resource finder object. Use it to configure your module.
    // If you are migrating from the old module, this is the function
    // equivalent to the "open" method.
    bool configure(yarp::os::ResourceFinder &rf)
    {
        count=0;
        // optional, attach a port to the module
        // so that messages received from the port are redirected
        // to the respond method
        handlerPort.open("/myModule");
        attach(handlerPort);
        return true;
    }
    // Interrupt function.
    bool interruptModule()
    {
        cout << "Interrupting your module, for port cleanup" << endl;
        return true;
    }
    // Close function, to perform cleanup.
    bool close()
    {
        // optional, close port explicitly
        cout << "Calling close function\n";
        handlerPort.close();
        return true;
    }
=======
        // Respective time, accelerated in one direction.
        double t_iter_;

        // Velocity.
        yarp::sig::Vector vel_;

        // User interface.
        WINDOW *win_w_, *win_a_, *win_s_, *win_d_;
        WINDOW *win_q_, *win_e_, *win_info_, *win_vel_;
>>>>>>> 9efb23a88150b355067ba3223d22360128c4404e
};

#endif