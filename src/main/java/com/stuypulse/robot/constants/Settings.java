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
import com.stuypulse.stuylib.network.SmartBoolean;
import com.stuypulse.stuylib.network.SmartNumber;
import com.stuypulse.stuylib.streams.IStream;
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

/*-
 * File containing tunable settings for every subsystem on the robot.
 *
 * We use StuyLib's SmartNumber / SmartBoolean in order to have tunable
 * values that we can edit on Shuffleboard.
 */
public interface Settings {

    Path DEPLOY_DIRECTORY = Filesystem.getDeployDirectory().toPath();

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

    }

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

    public static void reportWarning(String string) {
    }

    public interface Alignment {
        IStream RING_DISTANCE = new SmartNumber("Limelight/Ring Distance",
                50).filtered(Units::inchesToMeters);
        IStream PAD_DISTANCE = new SmartNumber("Limelight/Pad Distance", 217).filtered(Units::inchesToMeters);

        IStream APRIL_TAG_DISTANCE = new SmartNumber("Limelight/April Tag Distance", 100)
                .filtered(Units::inchesToMeters);

        double MIN_DISTANCE = Units.feetToMeters(1);
        double MAX_DISTANCE = Units.feetToMeters(54);

        double MIN_ALIGNMENT_TIME = 1.0;

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

            // Get PID Controller
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

        public interface Measurements {

            SmartNumber APRIL_TAG_16H52_2 = new SmartNumber("April Tag 2 Height", Units.inchesToMeters(88));
            SmartNumber APRIL_TAG_16H52_1 = new SmartNumber("April Tag 1 Height", Units.inchesToMeters(85.75));

            public interface Limelight {
                double HEIGHT = Units.feetToMeters(2) + Units.inchesToMeters(10);
                double DISTANCE = Units.feetToMeters(0);
                double PITCH = Units.degreesToRadians(23);
                double YAW = 0.0;

                // What angle error should make us start distance alignment
                SmartNumber MAX_ANGLE_FOR_MOVEMENT = new SmartNumber("Limelight/Max Angle For Distance", 3.0);

                SmartNumber MAX_ANGLE_ERROR = new SmartNumber("Limelight/Max Angle Error", 2);
                SmartNumber MAX_DISTANCE_ERROR = new SmartNumber("Limelight/Max Distance Error",
                        Units.inchesToMeters(6));

                SmartNumber MAX_VELOCITY = // THERE WAS AN ERROR WHERE THIS WOULD'NT CHECK WHEN MOVING BACKWARDS
                        new SmartNumber("Limelight/Max Velocity Error", Units.inchesToMeters(3));

                // How long it takes to stop aligning
                double DEBOUNCE_TIME = 0.2;
            }
        }
    }
}
