// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-


#ifndef __RECOVERY_HPP__
#define __RECOVERY_HPP__

#include <openrave-core.h>
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <time.h>       /* time_t, struct tm, time, localtime, asctime */

using namespace OpenRAVE;
using namespace std;

class Recovery{
  private:
    RobotBasePtr probot;
    EnvironmentBasePtr penv;
    ControllerBasePtr pcontrol;
    KinBody::LinkPtr plink;
    KinBody::JointPtr pjoint;

  public:
    bool init();
};

#endif  // __RECOVERY_HPP__

