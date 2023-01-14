package com.stuypulse.robot.commands.swerve;

import com.stuypulse.robot.subsystems.SwerveDrive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class SwerveDriveHome extends InstantCommand {
    public SwerveDriveHome(SwerveDrive swerve) {
        super(() -> swerve.reset(new Pose2d()), swerve);
    }
}
