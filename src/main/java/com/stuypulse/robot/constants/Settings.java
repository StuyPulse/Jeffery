/************************ PROJECT PHIL ************************/
/* Copyright (c) 2023 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.constants;

import com.stuypulse.stuylib.network.SmartNumber;

/*-
 * File containing tunable settings for every subsystem on the robot.
 *
 * We use StuyLib's SmartNumber / SmartBoolean in order to have tunable
 * values that we can edit on Shuffleboard.
 */

public interface Settings {
    public interface Arm {
        
        public interface Shoulder {
            double GEARING = 80;
            double LENGTH = 0.8; // meters
            double MAXANGLE = 90; 
            double MINANGLE = -90;
            double MASS = 7; 
            double WEIGHT = MASS * 9.81; 
            double JKG = 0.33 * MASS * (Math.pow(LENGTH, 2));

            double VEL_LIMIT = 30;
            double ACCEL_LIMIT = 10;

            double MIDGOAL = 0.5; // meters
            double HIGHGOAL = 0.5; // meters

            public interface PID {
                SmartNumber kP = new SmartNumber("kP", 1);
                SmartNumber kI = new SmartNumber ("kI", 0);
                SmartNumber kD = new SmartNumber("kD", 0.1);
            }
        

            public interface Feedforward {
                double kS = 0.1;
                double kA = 0.06;
                double kG = 3.382;
                double kV = 0.3;
            }
        }

        public interface Wrist {
            
            double GEARING = 80;
            double LENGTH = 0.3; // meters
            double MAXANGLE = 90; 
            double MINANGLE = -90;
            double MASS = 4;
            double WEIGHT = MASS * 9.81;
            double JKG = 0.33 * MASS * (Math.pow(LENGTH, 2));

            double VEL_LIMIT = 4;
            double ACCEL_LIMIT = 2;

            double MIDGOAL = 0.5; // meters
            double HIGHGOAL = 0.5; // meters
        
            public interface PID {
                SmartNumber kP = new SmartNumber("kP", 1);
                SmartNumber kI = new SmartNumber ("kI", 0);
                SmartNumber kD = new SmartNumber("kD", 0.1);
            }

            public interface Feedforward {
                double kS = 0.1;
                double kA = 0.05;
                double kG = 0.67;
                double kV = 0.1;
            }
        }        
    }
}
