package com.stuypulse.robot.subsystems;
import java.util.stream.Stream;

import com.stuypulse.robot.constants.Settings.Swerve.Chassis;
import com.stuypulse.robot.subsystems.modules.SwerveModule;
import com.stuypulse.stuylib.math.Vector2D;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class ISwerve extends SubsystemBase {

    /* MODULE API */
    public abstract Field2d getField();

    public abstract ChassisSpeeds getChassisSpeeds();

    public abstract Translation2d getVelocity();

    public abstract Translation2d getTranslation();

    public abstract SwerveModule[] getModules();

    public abstract Stream<SwerveModule> getModuleStream();

    public abstract SwerveModuleState[] getModuleStates();

    public abstract void reset(Pose2d pose);

    public abstract void setStates(Vector2D velocity, double omega, boolean fieldRelative);

    protected abstract void setStatesRetainAngle(ChassisSpeeds chassisSpeeds);

    public abstract void setStates(Vector2D velocity, double omega);


    /* GYRO API */
    public abstract Rotation2d getGyroAngle();

    /* ODOMETRY API */

    protected abstract void updatePose();

    public abstract Pose2d getPose();

    public abstract Rotation2d getAngle();

    public abstract SwerveDriveKinematics getKinematics();
}
