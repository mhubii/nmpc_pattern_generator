#include <stdio.h>
#include <yarp/os/all.h>

#include "reader.h"

int main() {
    KeyReader key_reader(4000);

    key_reader.start();

    bool done = false;
    double startTime = yarp::os::Time::now();
    const int SOME_TIME=10;
    while(!done)
    {
        if ((yarp::os::Time::now() - startTime) > SOME_TIME)
            done = true;
    }

    key_reader.stop();
}