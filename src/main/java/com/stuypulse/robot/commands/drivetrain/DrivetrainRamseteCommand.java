/* Copyright (c) 2021 StuyPulse Robotics. All rights reserved. */
/* This work is licensed under the terms of the MIT license */
/* found in the root directory of this project. */

package com.stuypulse.robot.commands.drivetrain;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPRamseteCommand;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Drivetrain.Motion;
import com.stuypulse.robot.subsystems.Drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.trajectory.Trajectory;

public class DrivetrainRamseteCommand extends PPRamseteCommand {

    private boolean resetPosition;
    private Trajectory trajectory;
    private Drivetrain drivetrain;

    public DrivetrainRamseteCommand(Drivetrain drivetrain, PathPlannerTrajectory trajectory) {
        super(
                trajectory,
                drivetrain::getPose,
                new RamseteController(),
                Motion.MOTOR_FEED_FORWARD,
                Motion.KINEMATICS,
                drivetrain::getWheelSpeeds,
                new PIDController(Motion.PID.P, Motion.PID.I, Motion.PID.D),
                new PIDController(Motion.PID.P, Motion.PID.I, Motion.PID.D),
                drivetrain::tankDriveVolts,
                drivetrain);

        this.resetPosition = true;
        this.trajectory = trajectory;
        this.drivetrain = drivetrain;
    }

    public DrivetrainRamseteCommand(Drivetrain drivetrain, String path) {
        this(drivetrain, PathPlanner.loadPath(path, new PathConstraints(Settings.Drivetrain.MAX_VELOCITY, Settings.Drivetrain.MAX_ACCELERATION)));
    }

    // public DrivetrainRamseteCommand(Drivetrain drivetrain, String... paths) {
    //     this(drivetrain, PathPlanner.loadPathGroup(, null, null)));
    // }

    // [DEFAULT] Resets the drivetrain to the begining of the trajectory
    public DrivetrainRamseteCommand robotRelative() {
        this.resetPosition = true;
        return this;
    }

    // Make the trajectory relative to the field
    public DrivetrainRamseteCommand fieldRelative() {
        this.resetPosition = false;
        return this;
    }

    @Override
    public void initialize() {
        super.initialize();

        drivetrain.setHighGear();

        if (resetPosition) {
            drivetrain.reset(trajectory.getInitialPose());
        }
    }
}