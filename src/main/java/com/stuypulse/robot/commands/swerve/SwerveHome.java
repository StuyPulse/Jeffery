package com.stuypulse.robot.commands.swerve;

import com.stuypulse.robot.subsystems.SwerveDrive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class SwerveHome extends InstantCommand {
    public SwerveHome(SwerveDrive swerve, Pose2d resetPose) {
        super(() -> swerve.reset(resetPose), swerve);
    }
}
