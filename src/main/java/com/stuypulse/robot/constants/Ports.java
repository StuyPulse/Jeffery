/************************ PROJECT PHIL ************************/
/* Copyright (c) 2023 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.constants;

/** This file contains the different ports of motors, solenoids and sensors */
public interface Ports {
    public interface Gamepad {
        int DRIVER = 0;
        int OPERATOR = 1;
        int DEBUGGER = 2;
    }
    
    public interface Swerve {
        public interface FrontRight {
            int DRIVE = -1;
            int TURN = -1;
            int ENCODER = -1;
        }

        public interface FrontLeft  {
            int DRIVE = -1;
            int TURN = -1;
            int ENCODER = -1;
        }

        public interface BackLeft {
            int DRIVE = -1;
            int TURN = -1;
            int ENCODER = -1;
        }
        
        public interface BackRight {
            int DRIVE = -1;
            int TURN = -1;
            int ENCODER = -1;
        }
    }
}
