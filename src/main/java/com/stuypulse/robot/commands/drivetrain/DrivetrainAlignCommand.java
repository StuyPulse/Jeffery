/************************ PROJECT DORCAS ************************/
/* Copyright (c) 2022 StuyPulse Robotics. All rights reserved.  */
/* This work is licensed under the terms of the MIT license.    */
/****************************************************************/

package com.stuypulse.robot.commands.drivetrain;

import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.angle.AngleController;
import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.streams.filters.IFilter;
import com.stuypulse.stuylib.streams.filters.LowPassFilter;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Alignment;
import com.stuypulse.robot.subsystems.ICamera;
import com.stuypulse.robot.subsystems.Drivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class DrivetrainAlignCommand extends SequentialCommandGroup {

    // protected final Controller distanceController;
    // protected final AngleController angleController;

    private final PathPlannerTrajectory trajectory;

    public DrivetrainAlignCommand(Drivetrain drivetrain, ICamera camera, Pose2d target) {
        // this.drivetrain = drivetrain;
        // this.camera = camera;

        // this.angleController = Alignment.Angle.getPID();
        // this.distanceController = Alignment.Speed.getPID();

        Pose2d currentPosition = drivetrain.getPose();
        this.trajectory = PathPlanner.generatePath(
            new PathConstraints(Settings.Drivetrain.MAX_VELOCITY/5, Settings.Drivetrain.MAX_ACCELERATION/3),
            new PathPoint(currentPosition.getTranslation(), currentPosition.getRotation()),
            new PathPoint(target.getTranslation(), target.getRotation())
        );

        addCommands(
            new DrivetrainRamseteCommand(drivetrain, trajectory)
                .withTimeout(Alignment.TIMEOUT)
                // .withInterrupt(() -> isAligned())
            );
    }
}