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

    public interface Drivetrain {
        int LEFT_TOP = 7;
        int LEFT_BOTTOM = 6;

        int RIGHT_TOP = 4;
        int RIGHT_BOTTOM = 3;

        int LEFT_ENCODER_A = 0;
        int LEFT_ENCODER_B = 1;
        int RIGHT_ENCODER_A = 2;
        int RIGHT_ENCODER_B = 3;

        int GEAR_SHIFT = 0;
    }

    public interface Pneumatics {
        int ANALOG_PRESSURE_SWITCH_PORT = 0;
        double ANALOG_PRESSURE_SWITCH_VOLTAGE_SUPPLY = 5.0;
    }
    
    public interface Swerve {
        public interface FrontRight {
            int DRIVE = 10;
            int TURN = 11;
            int ENCODER = 1;
        }

        public interface FrontLeft  {
            int DRIVE = 12;
            int TURN = 13;
            int ENCODER = 3;
        }

        public interface BackLeft {
            int DRIVE = 14;
            int TURN = 15;
            int ENCODER = 2;
        }
        
        public interface BackRight {
            int DRIVE = 16;
            int TURN = 17;
            int ENCODER = 0;
        }
    }
}
