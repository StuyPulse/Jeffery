package com.stuypulse.robot.subsystems;
import java.util.stream.Stream;
import com.stuypulse.robot.subsystems.modules.SwerveModule;
import com.stuypulse.stuylib.math.Vector2D;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class ISwerve extends SubsystemBase {

    /* MODULE API */
    public abstract Field2d getField();

    public abstract SwerveModule getModule();

    public abstract ChassisSpeeds getChassisSpeed();

    public abstract Translation2d getVelocity();

    public abstract Translation2d getTranslation();

    public abstract SwerveModule[] getModules();

    public abstract Stream<SwerveModule> getModuleStream();

    public abstract SwerveModulePosition[] getModuleStates();

    public abstract void reset();

    public abstract void setStates(Vector2D velocity, double omega, boolean fieldRelative);

    public abstract void setStatesRetainAngle();

    public abstract void setStates(Vector2D velocity, double omega);

    public abstract void setStates(ChassisSpeeds robotSpeed);

    /* GYRO API */
    public abstract Rotation2d getGyroAngle();

    /* ODOMETRY API */

    public abstract void updatePose();

    public abstract Pose2d getPose();

    public abstract Rotation2d getAngle();

    public abstract SwerveDriveKinematics getKinematics();
}
