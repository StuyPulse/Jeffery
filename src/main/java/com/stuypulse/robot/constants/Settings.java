/************************ PROJECT PHIL ************************/
/* Copyright (c) 2023 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.constants;


/*-
 * File containing tunable settings for every subsystem on the robot.
 *
 * We use StuyLib's SmartNumber / SmartBoolean in order to have tunable
 * values that we can edit on Shuffleboard.
 */

public interface Settings {
    public interface Arm {
        
        public interface ArmArm {
            double GEARING = 80;
            double WEIGHT = 7;
            double LENGTH = 0.8; // meters
            double MAXANGLE = 90; 
            double MINANGLE = -90;
            double MASS = 15; // micrograms
            double JKG = 9;

            double VEL_LIMIT = 5;
            double ACCEL_LIMIT = 3;

            double MIDGOAL = 0.5; // meters
            double HIGHGOAL = 0.5; // meters

            public interface PID {
                double kP = 0.5;
                double kI = 0.5;
                double kD = 0.5;
            }
        

            public interface Feedforward {
                double kS = 0.5;
                double kA = 0.5;
                double kG = 0.5;
                double kV = 0.5;
            }
        }

        public interface Wrist {
            
            double GEARING = 80;
            double WEIGHT = 5;
            double LENGTH = 0.3; // meters
            double MAXANGLE = 90; 
            double MINANGLE = -90;
            double MASS = 3; // micrograms
            double JKG = 15;

            double VEL_LIMIT = 4;
            double ACCEL_LIMIT = 2;

            double MIDGOAL = 0.5; // meters
            double HIGHGOAL = 0.5; // meters
        
            public interface PID {
                double kP = 0.5;
                double kI = 0.5;
                double kD = 0.5;
            }

            public interface Feedforward {
                double kS = 0.5;
                double kA = 0.5;
                double kG = 0.5;
                double kV = 0.5;
            }
        }        
    }
}
