/************************ PROJECT PHIL ************************/
/* Copyright (c) 2023 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.constants;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.auto.PIDConstants;
import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.math.Vector2D;
import com.stuypulse.stuylib.network.SmartAngle;
import com.stuypulse.stuylib.network.SmartBoolean;
import com.stuypulse.stuylib.network.SmartNumber;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

/*-
 * File containing tunable settings for every subsystem on the robot.
 *
 * We use StuyLib's SmartNumber / SmartBoolean in order to have tunable
 * values that we can edit on Shuffleboard.
 */
public interface Settings {
    double DT = 0.02;

    public static Vector2D vpow(Vector2D vec, double power) {
        return vec.mul(Math.pow(vec.magnitude(), power - 1));
    }
    
    public interface Driver {
        SmartNumber DEADBAND = new SmartNumber("Driver Settings/Deadband", 0.05);
        SmartNumber MAX_TELEOP_SPEED = new SmartNumber("Driver Settings/Max Speed", 4.2);
        SmartNumber MAX_TELEOP_ACCEL = new SmartNumber("Driver Settings/Max Accleration", 7);
        SmartNumber MAX_TELEOP_TURNING = new SmartNumber("Driver Settings/Max Turning", 6.1);

        public interface Drive {
            SmartNumber RC = new SmartNumber("Driver Settings/Drive/RC", 0.25);
            SmartNumber POWER = new SmartNumber("Driver Settings/Drive/Power", 2);
        }

        public interface Turn {
            SmartNumber RC = new SmartNumber("Driver Settings/Turn/RC", 0.15);
            SmartNumber POWER = new SmartNumber("Driver Settings/Turn/Power", 2);
        }

    }

    public interface Swerve {
        double MIN_MODULE_VELOCITY = 0.05;

        public interface Chassis {
            double WIDTH = Units.inchesToMeters(29.0);
            double HEIGHT = Units.inchesToMeters(29.0);
            double MAX_SPEED = 4.2;
        }

        public interface Drive {
            SmartNumber kS = new SmartNumber("Swerve/Drive/kS", 0.17335);
            SmartNumber kV = new SmartNumber("Swerve/Drive/kV", 2.7274);
            SmartNumber kA = new SmartNumber("Swerve/Drive/kA", 0.456);

            SmartNumber kP = new SmartNumber("Swerve/Drive/kP", 1.3);
            SmartNumber kI = new SmartNumber("Swerve/Drive/kI", 0.0);
            SmartNumber kD = new SmartNumber("Swerve/Drive/kD", 0.0);
        }

        public interface Turn {
            SmartNumber kP = new SmartNumber("Swerve/Turn/kP", 3.5);
            SmartNumber kI = new SmartNumber("Swerve/Turn/kI", 0.0);
            SmartNumber kD = new SmartNumber("Swerve/Turn/kD", 0.1);

            double kS = 0.14;
            double kV = 0.25;
            double kA = 0.007;
        }

        public interface FrontRight {
            String ID = "Front Right";
            SmartAngle ABSOLUTE_OFFSET = new SmartAngle(ID + "/Absolute Offset", Angle.fromDegrees(143));
            Translation2d MODULE_OFFSET = new Translation2d(Chassis.WIDTH * +0.5, Chassis.HEIGHT * -0.5);
        }

        public interface FrontLeft {
            String ID = "Front Left";
            SmartAngle ABSOLUTE_OFFSET = new SmartAngle(ID + "/Absolute Offset", Angle.fromDegrees(36));
            Translation2d MODULE_OFFSET = new Translation2d(Chassis.WIDTH * +0.5, Chassis.HEIGHT * +0.5);
        }

        public interface BackLeft {
            String ID = "Back Left";
            SmartAngle ABSOLUTE_OFFSET = new SmartAngle(ID + "/Absolute Offset", Angle.fromDegrees(-80.5));
            Translation2d MODULE_OFFSET = new Translation2d(Chassis.WIDTH * -0.5, Chassis.HEIGHT * +0.5);
        }

        public interface BackRight {
            String ID = "Back Right";
            SmartAngle ABSOLUTE_OFFSET = new SmartAngle(ID + "/Absolute Offset", Angle.fromDegrees(142.3));
            Translation2d MODULE_OFFSET = new Translation2d(Chassis.WIDTH * -0.5, Chassis.HEIGHT * -0.5);
        }

        public interface Encoder {
            public interface Drive {
                double WHEEL_DIAMETER = Units.inchesToMeters(4.0);
                double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;

                public interface Stages {
                    // input / output
                    double FIRST = 16.0 / 48.0;
                    double SECOND = 28.0 / 16.0;
                    double THIRD = 15.0 / 60.0;
                }

                double GEAR_RATIO = Stages.FIRST * Stages.SECOND * Stages.THIRD;

                double POSITION_CONVERSION = WHEEL_CIRCUMFERENCE * GEAR_RATIO;
                double VELOCITY_CONVERSION = POSITION_CONVERSION / 60.0;
            }

            public interface Turn {
                double GEAR_RATIO = 1.0 / 12.8;
                double POSITION_CONVERSION = GEAR_RATIO * 2 * Math.PI;
                double VELOCITY_CONVERSION = POSITION_CONVERSION / 60.0;
            }
        }
        
        public interface Motion {
            PathConstraints CONSTRAINTS = new PathConstraints(3.5, 2);

            PIDConstants XY = new PIDConstants(1, 0, 0.1);
            PIDConstants THETA = new PIDConstants(10, 0, 0.1);
        }
    }
}
