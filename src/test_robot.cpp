#include <yarp/dev/all.h>
#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/eigen/Eigen.h>   
#include <Eigen/Core>
#include <iostream>

#include "utils.h"

class CheckRateThread : public yarp::os::RateThread
{
    public: CheckRateThread(int period) : RateThread(period) {

                port_.open("/check_rate_thread");
            };
      
            ~CheckRateThread() { 

                port_.close();
            };

    private: virtual void run() { 

                yarp::sig::Vector* read = port_.read(false);

                if (read != YARP_NULLPTR) {

                    std::cout << yarp::eigen::toEigen(*read) << std::endl;
                }

                // double t1;
                // double t2;
 
                for (int i = 0; i < 10; i++) {
                    
                    // t1 = yarp::os::Time::now();

                    // TODO: how to assure that the loop takes a specific time and not the delay?
                    // t2 = yarp::os::Time::now();
                    // yarp::os::Time::delay(0.03); // - t2 - t1);
                }
             };

             yarp::os::BufferedPort<yarp::sig::Vector> port_;
};

int main() {

    // Try to print the address of a buffered port in yarp and read from the app.
    yarp::os::Network yarp;

    if (!yarp.checkNetwork()) {

        std::cout << "No yarpserver is running. Please run a yarpserver." << std::endl;
        std::exit(1);
    }

    yarp::os::BufferedPort<yarp::os::Bottle> read;
    read.open("/read");

    yarp::os::BufferedPort<yarp::os::Bottle> write;
    write.open("/write");

    std::cout << read.where().getPort() << std::endl;
    std::cout << write.where().getPort() << std::endl;

    yarp::sig::Vector rec;

    // std::cout << "bottle is \n" << read.read(true)->toString() << std::endl;;

    read.read(true)->get(0).asList()->write(rec);
    std::cout << rec.toString() << std::endl;

    read.close();
    write.close();




    // // Check boolean operations of eigen matrices.
    // Eigen::Matrix2d one;
    // one << 1, 2, 3, 4;

    // Eigen::Matrix2d two;
    // two << 2, 3, 4, 3;

    // bool comp = (one.array() > two.array()).any();

    // std::cout << comp << std::endl;




    // // Check a run thread that reads slower than its port gets written to.
    // yarp::os::Network yarp;
    
    // CheckRateThread thread(200);

    // yarp::os::BufferedPort<yarp::sig::Vector> input_port;
    // input_port.open("/input_port");

    // yarp::os::Network::connect("/input_port", "/check_rate_thread");

    // thread.start();

    // for (int i = 0; i < 10; i++) {

    //     yarp::sig::Vector input(1);
    //     input(0) = i;

    //     input_port.prepare() = input;
    //     input_port.write(true);

    //     yarp::os::Time::delay(0.2);
    // }

    // thread.stop();

    // input_port.close();


    // Check storage order of yarp matrices.
    // // Yarp matrix.
    // yarp::sig::Matrix yarp(2, 2);
    // yarp.zero();

    // // Eigen matrix.
    // Eigen::MatrixXd eigen(2, 2);
    // eigen.setConstant(5);
    // eigen(0, 1) = 0;

    // // Copy data from Eigen to Yarp matrix.
    // Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(yarp.data(), eigen.rows(), eigen.cols()) = eigen;

    // // Show results.
    // std::cout << "yarp:\n" << yarp::eigen::toEigen(yarp) << std::endl;
    // std::cout << "eigen:\n" << eigen << std::endl;

    // std::cout << "yarp parts:\n" << yarp(0,0) << " " << yarp(0,1) << "\n" << yarp(1,0) << " " << yarp(1,1) << std::endl;
    // std::cout << "yarp parts:\n" << eigen(0,0) << " " << eigen(0,1) << "\n" << eigen(1,0) << " " << eigen(1,1) << std::endl;




    // Test to connect one port to many others.
    // yarp::os::Network yarp;

    // // Buffered port to send status.
    // yarp::os::BufferedPort<yarp::os::Bottle> port;
    // port.open("/writer/info");
    // yarp::os::BufferedPort<yarp::os::Bottle> port_1;
    // port_1.open("/writer/info_1");
    // yarp::os::BufferedPort<yarp::os::Bottle> port_2;
    // port_2.open("/writer/info_2");

    // // Connect ports.
    // yarp::os::Network::connect("/writer/info", "/writer/info_1");
    // yarp::os::Network::connect("/writer/info", "/writer/info_2");

    // yarp::os::Bottle& bottle = port.prepare();
    // yarp::os::Property& dict = bottle.addDict();

    // dict.put("hi", 1);
    // port.write();

    // yarp::os::Bottle* bottle_1 = port_1.read();
    // yarp::os::Value prop_1 = bottle_1->pop();
    // std::cout << prop_1.asDict()->find("hi").asInt() << std::endl;

    // yarp::os::Bottle* bottle_2 = port_2.read();
    // yarp::os::Value prop_2 = bottle_2->pop();
    // std::cout << prop_2.asDict()->find("hi").asInt() << std::endl;

    // // Send status.
    // yarp::os::Bottle& bottle = port.prepare();
    // yarp::os::Property& dict = bottle.addDict();

    // dict.put("InitialPositionStatus", DONE);
    // port.write();
    // yarp::os::Time::delay(2);

    // dict.put("InitialPositionStatus", MOVING);
    // port.write();
    // yarp::os::Time::delay(2);

    // dict.put("InitialPositionStatus", DONE);
    // port.write();
    // yarp::os::Time::delay(2);

    // port.close();
    // port_1.close();
    // port_2.close();

    return 0;
}