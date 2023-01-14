package com.stuypulse.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class IDrivetrain extends SubsystemBase {
    
    public abstract Pose2d getPose2d();

    public abstract Pose2d setPose2d();

    public abstract DifferentialDriveWheelSpeeds getWheelSpeeds();

    public abstract void tankDriveVolts(double leftVolts, double rightVolts);

    public abstract void arcadeDrive(double speed, double angle);
}
