#include <stdio.h>
#include <yarp/os/all.h>
#include <Eigen/Dense>
#include <iostream>

#include "reader.h"

<<<<<<< HEAD
#include <ncurses.h>
#include <SDL2/SDL.h>

using namespace std;

int main( int argc, char *argv[] ){

    /* Event structure */
    SDL_Init(SDL_INIT_EVERYTHING);
    SDL_Event event;

    /* Check for events */
    while(SDL_PollEvent(&event)){  /* Loop until there are no events left on the queue */
        switch(event.type){  /* Process the appropiate event type */
            case SDL_KEYDOWN:  /* Handle a KEYDOWN event */         
                printf("Oh! Key press\n");
                break;
            case SDL_MOUSEMOTION:
                printf("Mouse moving!\n");
                break;
            default: /* Report an unhandled event */
                printf("I don't know what this event is!\n");
        }
    }
    SDL_Quit();
=======
int main(int argc, char * argv[]) {
    yarp::os::Network yarp;


    int period = 10;
    // ReadJoints rj(1000);
    ReadCameras rc(100);

    // rj.start();
    rc.start();
    yarp::os::Time::delay(10);
    rc.stop();
    // rj.stop();

    return 0;
>>>>>>> 39be5a8d8353015eacc8d54cbfd2b67c31d55566
}