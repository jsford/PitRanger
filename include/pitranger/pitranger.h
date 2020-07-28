#pragma once

#include "pr_imu.h"
#include "pr_motor.h"
#include "pr_remote.h"
#include "pr_time.h"
//#include "pr_pitcam.h"
//#include "pr_navcam.h"

namespace pr {

class Robot {
    public:
        Robot();

        //Pose get_pose() const;

        //bool drive_arc(const DriveArc& arc);


    private:
        //MissionClock            clock;
        RemoteControl                rc;
        MotorController    drive_motors;
        //PitCamera          pit_camera;
        //NavCamera        front_stereo;
        //NavCamera         back_stereo;
        //Logger                 logger;
};


}
