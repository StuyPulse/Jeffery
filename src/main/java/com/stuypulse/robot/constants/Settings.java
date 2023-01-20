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
        double GEARING = -1;
        double WEIGHT = -1;
        double LENGTH = -1;
        double MAXANGLE = -1;
        double MINANGLE = -1;
        double MASS = -1;

        double VEL_LIMIT = 1;
        double ACCEL_LIMIT = 1;

        double MIDGOAL = -1;
        double HIGHGOAL = -1;

        public interface ArmArm {
            public interface PID {
                double kP = -1;
                double kI = -1;
                double kD = -1;
            }

            public interface Feedforward {
                double kS = -1;
                double kA = -1;
                double kG = -1;
                double kV = -1;
            }
        }

        public interface Wrist {
            public interface PID {
                double kP = -1;
                double kI = -1;
                double kD = -1;
            }

            public interface Feedforward {
                double kS = -1;
                double kA = -1;
                double kG = -1;
                double kV = -1;
            }
        }

        
    }
}
