/************************ PROJECT PHIL ************************/
/* Copyright (c) 2023 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.constants;

import com.stuypulse.stuylib.network.SmartBoolean;
import com.stuypulse.stuylib.network.SmartNumber;

/*-
 * File containing tunable settings for every subsystem on the robot.
 *
 * We use StuyLib's SmartNumber / SmartBoolean in order to have tunable
 * values that we can edit on Shuffleboard.
 */
public interface Settings {
    public interface Arm {
        double GEARING = -1;
        double WEIGHT = -1;
        double LENGTH = -1;
        double MAXANGLE = -1;
        double MINANGLE = -1;
        double MASS = -1;

        double MIDGOAL = -1;
        double HIGHGOAL = -1;

        public interface PID {
            double kP = -1;
            double kI = -1;
            double kD = -1;
        }
    }
}
