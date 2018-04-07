#ifndef READ_H_
#define READ_H_

// Wrapper class for YARP to read from ports.
//
// Implemented by Martin Huber.
class Read
{
    public:
        Read();

        ReadFromCamera();

        ReadFromJoints();
};

#endif