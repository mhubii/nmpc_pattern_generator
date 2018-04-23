#include "reader.h"

#include <iostream>

KeyReader::KeyReader() 
    : t_iter_(yarp::os::Time::now()), 
      acc_w_( 0.1, 0.,  0. ),
      acc_a_( 0. , 0.,  0.1),
      acc_s_(-0.1, 0.,  0. ),
      acc_d_( 0. , 0., -0.1),
      vel_(3) {

  // Open port.
  port.open("/vel");

  // Set accelerations and velocity.
  vel_.zero();

  // User interface.
  initscr();
  timeout(10);
  noecho();
  cbreak();
  curs_set(0);


  win_w_ = newwin( 3, 6,  2, 10);
  win_a_ = newwin( 3, 6,  6,  2);
  win_s_ = newwin( 3, 6,  6, 10);
  win_d_ = newwin( 3, 6,  6, 18);
  win_q_ = newwin( 3, 6,  2,  2);
  win_e_ = newwin( 3, 6,  2, 18);
  win_info_ = newwin(18, 44, 2, 28);
  win_vel_ = newwin(6, 20, 15, 2);

  start_color();
  init_pair(1, COLOR_WHITE, COLOR_BLUE);
  init_pair(2, COLOR_BLUE, COLOR_WHITE);
  init_pair(3, COLOR_WHITE, COLOR_YELLOW);

  bkgd(COLOR_PAIR(1));
  wbkgd(win_w_, COLOR_PAIR(2));
  wbkgd(win_a_, COLOR_PAIR(2));
  wbkgd(win_s_, COLOR_PAIR(2));
  wbkgd(win_d_, COLOR_PAIR(2));
  wbkgd(win_q_, COLOR_PAIR(3));
  wbkgd(win_e_, COLOR_PAIR(3));
  wbkgd(win_info_, COLOR_PAIR(1));
  wbkgd(win_vel_, COLOR_PAIR(1));

  mvwaddstr(win_w_, 1, 2, "w");
  mvwaddstr(win_a_, 1, 2, "a");
  mvwaddstr(win_s_, 1, 2, "s");
  mvwaddstr(win_d_, 1, 2, "d");
  mvwaddstr(win_q_, 1, 2, "q");
  mvwaddstr(win_e_, 1, 2, "e");
  mvwaddstr(win_info_, 0, 0, "Hello,\n\n"
                          "I am the keyboard user interface of the heicub robot.\n\n"
                          "Use w and s to accelerate forwards and backwards.\n\n"
                          "Use a and d to accelerate angular left and right.\n\n"
                          "Only one pressed key is used to accelerate, leaving all other velocities untouched, except for the opposite ones.\n\n"
                          "For an emergency stop press e.\n\n"
                          "Press q to quit this interface."); 
  mvwaddstr(win_vel_, 0, 0, ("Current velocity:\n\n"
                             "v_x:   " + std::to_string(vel_(0)) + "\n"
                             "v_y:   " + std::to_string(vel_(1)) + "\n"
                             "v_ang: " + std::to_string(vel_(2))).c_str());

  refresh();
  wrefresh(win_w_);
  wrefresh(win_a_);
  wrefresh(win_s_);
  wrefresh(win_d_);
  wrefresh(win_q_);
  wrefresh(win_e_);
  wrefresh(win_info_);
  wrefresh(win_vel_);

  // Read incomming commands.
  ReadCommands();
}

KeyReader::~KeyReader() {

  // Set velocity to zero before closing.
  vel_.zero();
  WriteToPort();

  // Close port.
  port.close();
    
  // Release the user interface.
  delwin(win_w_);
  delwin(win_a_);
  delwin(win_s_);
  delwin(win_d_);
  delwin(win_q_);
  delwin(win_e_);
  delwin(win_info_);
  delwin(win_vel_);

  clrtoeol();
  refresh();
  endwin();
}

void KeyReader::ReadCommands() {

    // Read the pressed keys, set the velocities
    // and write them to the port.
    int ch = 0;
    double dt = 0.;

    do {                
        // Read input periodically.
        ch = getch();
                
        // Measure time between successive inputs.
        t_iter_ = yarp::os::Time::now() - t_iter_;
        dt = t_iter_;
        t_iter_ = yarp::os::Time::now();

        switch(ch)
        {
            case 'w': {
                SetVelocity(acc_w_, dt);
                WriteToPort();
                break;}
            case 'a':
                SetVelocity(acc_a_, dt);
                WriteToPort();
                break;
            case 's':
                SetVelocity(acc_s_, dt);
                WriteToPort();
                break;
            case 'd':
                SetVelocity(acc_d_, dt);
                WriteToPort();
                break;
            case 'e':
                vel_.zero();
                WriteToPort();
                break;
        }

        // Update user interface.  
        mvwaddstr(win_vel_, 0, 0, ("Current velocity:\n\n"
                                   "v_x:   " + std::to_string(vel_(0)) + "\n"
                                   "v_y:   " + std::to_string(vel_(1)) + "\n"
                                   "v_ang: " + std::to_string(vel_(2))).c_str());
        wrefresh(win_vel_);

    } while (ch != 'q' && ch != 'Q'); // Exit if q or Q is pressed.
}

void KeyReader::SetVelocity(Eigen::Vector3d& acc, double t) {

    // Determine the new velocity, given the previous one,
    // and the acceleration.
    Eigen::Map<Eigen::VectorXd> vel = yarp::eigen::toEigen(vel_);
    vel += acc*t;
}

void KeyReader::WriteToPort() {
    
    // Write the velocities to the output port.
    yarp::sig::Vector& data = port.prepare();
    data = port.prepare();
    data = vel_;
    port.write();
}
