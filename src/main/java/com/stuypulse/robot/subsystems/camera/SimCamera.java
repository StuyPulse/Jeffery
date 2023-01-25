package com.stuypulse.robot.subsystems.camera;

import com.stuypulse.robot.constants.Settings.Field;
import com.stuypulse.robot.subsystems.Drivetrain;
import com.stuypulse.robot.subsystems.ICamera;
import com.stuypulse.stuylib.math.Angle;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SimCamera extends ICamera {

    Drivetrain drivetrain;

    private static final double MAX_FOV = 27.0;

    public SimCamera(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }

    @Override
    public double getLatency() {
        return 0.0;
    }

    @Override
    public double getDistance() {
        if (!hasTarget()) {
            return 0.0;
        }
        return 0;
    }

    private Angle getRawHorizontalOffset() {
        return Angle.kZero;
        // return Angle.fromRotation2d(
        //     drivetrain.getPose().getTranslation().minus(Field.HUB)
        //         .getAngle().minus(new Rotation2d(drivetrain.getGyroAngle())));
    }

    @Override
    public Angle getHorizontalOffset() {
        if (hasTarget())
            return getRawHorizontalOffset();
        
        return Angle.kZero;
    }

    @Override
    public boolean hasTarget() {
        return Math.abs(getRawHorizontalOffset().toDegrees()) <= MAX_FOV;
    }

    @Override
    public Pose2d getPose2d() {
        return drivetrain.getPose();
    } 

    @Override 
    public Pose3d getPose3d() {
        return new Pose3d();
    }

    @Override
    public void simulationPeriodic() {
        SmartDashboard.putBoolean("Camera/Has Target", hasTarget());
        SmartDashboard.putNumber("Camera/Raw Horiz Offset (deg)", getRawHorizontalOffset().toDegrees());
    }
    
}