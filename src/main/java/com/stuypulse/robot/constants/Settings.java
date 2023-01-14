/************************ PROJECT PHIL ************************/
/* Copyright (c) 2023 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.constants;

import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.angle.AngleController;
import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.math.SLMath;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.auto.PIDConstants;
import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.math.Vector2D;
import com.stuypulse.stuylib.network.SmartAngle;
import com.stuypulse.stuylib.network.SmartNumber;
import com.stuypulse.stuylib.streams.filters.IFilterGroup;
import com.stuypulse.stuylib.streams.filters.LowPassFilter;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;

import java.nio.file.Path;
import edu.wpi.first.wpilibj.Filesystem;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

/*-
 * File containing tunable settings for every subsystem on the robot.
 *
 * We use StuyLib's SmartNumber / SmartBoolean in order to have tunable
 * values that we can edit on Shuffleboard.
 */
public interface Settings {

    Pose2d STARTING_POSE = new Pose2d(Units.inchesToMeters(130), Units.inchesToMeters(16) + 0.89, new Rotation2d(Math.toRadians(180)));

    Path DEPLOY_DIRECTORY = Filesystem.getDeployDirectory().toPath();

    public interface Drivetrain {
        // If speed is below this, use quick turn
        SmartNumber BASE_TURNING_SPEED = new SmartNumber("Driver Settings/Base Turn Speed", 0.4);

        // Low Pass Filter and deadband for Driver Controls
        SmartNumber SPEED_DEADBAND = new SmartNumber("Driver Settings/Speed Deadband", 0.0);
        SmartNumber ANGLE_DEADBAND = new SmartNumber("Driver Settings/Turn Deadband", 0.0);

        SmartNumber SPEED_POWER = new SmartNumber("Driver Settings/Speed Power", 1.0);
        SmartNumber ANGLE_POWER = new SmartNumber("Driver Settings/Turn Power", 1.0);

        SmartNumber SPEED_FILTER = new SmartNumber("Driver Settings/Speed Filtering", 0.2);
        SmartNumber ANGLE_FILTER = new SmartNumber("Driver Settings/Turn Filtering", 0.02);

        // Current Limit for the motors
        int CURRENT_LIMIT = 40;

        // If the motors are inverted
        boolean IS_INVERTED = true;

        // The voltage multipliers for each side
        double RIGHT_VOLTAGE_MUL = -1.0;
        double LEFT_VOLTAGE_MUL = 1.0;

        // Width of the robot
        double TRACK_WIDTH = Units.inchesToMeters(27);

        double MAX_VELOCITY = Units.feetToMeters(12.0); // 12 ft/s
        double MAX_ACCELERATION = 3.0;

        interface Motion {

            DifferentialDriveKinematics KINEMATICS = new DifferentialDriveKinematics(TRACK_WIDTH);

            SimpleMotorFeedforward MOTOR_FEED_FORWARD =
                    new SimpleMotorFeedforward(FeedForward.S, FeedForward.V, FeedForward.A);

            interface FeedForward {
                double S = 0.367;
                double V = 2.07;
                double A = 0.47;
            }

            interface PID {
                double P = 0.00337;
                double I = 0;
                double D = 0;
            }
        }

        public interface Odometry {
            Translation2d STARTING_TRANSLATION = new Translation2d();
            Rotation2d STARTING_ANGLE = new Rotation2d();

            Pose2d STARTING_POSITION = new Pose2d(STARTING_TRANSLATION, STARTING_ANGLE);
        }

        // Encoder Constants
        public interface Encoders {
            double WHEEL_DIAMETER = Units.inchesToMeters(6);
            double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;

            double LOW_GEAR_DISTANCE_PER_ROTATION = WHEEL_CIRCUMFERENCE * (1.0 / 16.71);
            double HIGH_GEAR_DISTANCE_PER_ROTATION = WHEEL_CIRCUMFERENCE * (1.0 / 7.73);
            double LEFT_YEILD = 1.0;
            double RIGHT_YEILD = -1.0;
        }
    }

    public interface Limelight {
        int[] PORTS = {5800, 5801, 5802, 5803, 5804, 5805};
        double CAMERA_TO_CENTER = Units.inchesToMeters(14.0);
        Angle CAMERA_PITCH = Angle.fromDegrees(28);
        double CAMERA_HEIGHT = Units.inchesToMeters(32);
    }

    public interface Field {
        double HUB_HEIGHT = Units.feetToMeters(8) + Units.inchesToMeters(9);
        double HUB_TO_CENTER = Units.feetToMeters(2.0);
        Translation2d HUB = new Translation2d(Units.feetToMeters(12.8), 0);
    }

    public interface Alignment {

        // Pose2d TARGET_POSE = new Pose2d(6, -3, new Rotation2d());

        Pose2d TARGET_POSE = new Pose2d(6, 3, new Rotation2d());


        double MIN_ALIGNMENT_TIME = 1.0;

        double TIMEOUT = 15;

        SmartNumber SPEED_ADJ_FILTER = new SmartNumber("Drivetrain/Alignment/Speed Adj RC", 0.1);
        SmartNumber FUSION_FILTER = new SmartNumber("Drivetrain/Alignment/Fusion RC", 0.3);
        SmartNumber SENSOR_FUSION_RC = new SmartNumber("Drivetrain/Alignment/Sensor Fusion RC", 0.4);

        SmartNumber AUTOTUNE_P = new SmartNumber("Drivetrain/Alignment/Auto Tune/P", 0.8);
        SmartNumber AUTOTUNE_I = new SmartNumber("Drivetrain/Alignment/Auto Tune/I", 0.0);
        SmartNumber AUTOTUNE_D = new SmartNumber("Drivetrain/Alignment/Auto Tune/D", 0.1);

        public interface Speed {

            // Preset PID Values
            SmartNumber P = new SmartNumber("Drivetrain/Alignment/Speed/P", 0.984252);
            SmartNumber I = new SmartNumber("Drivetrain/Alignment/Speed/I", 0);
            SmartNumber D = new SmartNumber("Drivetrain/Alignment/Speed/D", 0.0656168);

            // Get PID Controller
            public static Controller getPID() {
                return new PIDController(P, I, D)
                        .setOutputFilter(new IFilterGroup(SLMath::clamp, new LowPassFilter(OUT_SMOOTH_FILTER)));
            }

            // Bang Bang speed when measuring PID Values
            double BANGBANG_SPEED = 0.5;

            // Low Pass Filter Time Constant for controller
            SmartNumber IN_SMOOTH_FILTER = new SmartNumber("Drivetrain/Alignment/Speed/In Filter", 0);
            SmartNumber OUT_SMOOTH_FILTER = new SmartNumber("Drivetrain/Alignment/Speed/Out Filter", 0.2);

            // Max speed for limelight to move
            double LIMELIGHT_MAX_SPEED = 0.8;

            // What is an acceptable error
            double MAX_SPEED_ERROR = Units.inchesToMeters(3.0);
            double MAX_SPEED_VEL = Units.inchesToMeters(9.0);
        }

        public interface Angle {
            // Preset PID Values
            SmartNumber P = new SmartNumber("Drivetrain/Alignment/Angle/P", 0.022);
            SmartNumber I = new SmartNumber("Drivetrain/Alignment/Angle/I", 0);
            SmartNumber D = new SmartNumber("Drivetrain/Alignment/Angle/D", 0.0023);

            public static AngleController getPID() {
                return new AnglePIDController(P, I, D)
                        .setOutputFilter(new IFilterGroup(SLMath::clamp, new LowPassFilter(OUT_SMOOTH_FILTER)));
            }

            // Bang Bang speed when measuring PID Values
            double BANGBANG_SPEED = 0.4;

            // Low pass Filter Time Constant for controller
            SmartNumber IN_SMOOTH_FILTER = new SmartNumber("Drivetrain/Alignment/Angle/In Filter", 0.00);
            SmartNumber OUT_SMOOTH_FILTER = new SmartNumber("Drivetrain/Alignment/Angle/Out Filter", 0.06);

            // What is an acceptable error
            double MAX_ANGLE_ERROR = 3;
            double MAX_ANGLE_VEL = 20.0;
        }
   }

    double DT = 0.02;

    public static Vector2D vpow(Vector2D vec, double power) {
        return vec.mul(Math.pow(vec.magnitude(), power - 1));
    }
    
    public interface Driver {
        
        // If speed is below this, use quick turn
        SmartNumber BASE_TURNING_SPEED = new SmartNumber("Driver Settings/Base Turn Speed", 0.45);

        // Low Pass Filter and deadband for Driver Controls
        SmartNumber SPEED_DEADBAND = new SmartNumber("Driver Settings/Speed Deadband", 0.00);
        SmartNumber ANGLE_DEADBAND = new SmartNumber("Driver Settings/Turn Deadband", 0.00);

        SmartNumber SPEED_POWER = new SmartNumber("Driver Settings/Speed Power", 2.0);
        SmartNumber ANGLE_POWER = new SmartNumber("Driver Settings/Turn Power", 1.0);

        SmartNumber SPEED_FILTER = new SmartNumber("DriPver Settings/Speed Filtering", 0.125);
        SmartNumber ANGLE_FILTER = new SmartNumber("Driver Settings/Turn Filtering", 0.005);
    
        
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

    public interface AlignmentCommand{
        
        public interface Translation {
            SmartNumber P = new SmartNumber("Alignment/Translation/kP", 2.5);
            SmartNumber I = new SmartNumber("Alignment/Translation/kI", 0);
            SmartNumber D = new SmartNumber("Alignment/Translation/kD", 0);
        }
        public interface Rotation {
            SmartNumber P = new SmartNumber("Alignment/Rotation/kP", 1);
            SmartNumber I = new SmartNumber("Alignment/Rotation/kI", 0);
            SmartNumber D = new SmartNumber("Alignment/Rotation/kD", 0);
        }
    }
}
