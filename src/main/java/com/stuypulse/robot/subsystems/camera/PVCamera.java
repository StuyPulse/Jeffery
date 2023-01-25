package com.stuypulse.robot.subsystems.camera;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonPipelineResult;

import com.stuypulse.robot.constants.Settings.Field;
import com.stuypulse.robot.constants.Settings.Limelight;
import com.stuypulse.robot.subsystems.ICamera;
import com.stuypulse.stuylib.math.Angle;

import edu.wpi.first.math.ComputerVisionUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PVCamera extends ICamera {

    private final PhotonCamera camera;
    private PhotonPipelineResult result;

    public PVCamera(String cameraName) {
        camera = new PhotonCamera(cameraName);
        result = null;

        PortForwarder.add(5800, "photonvision.local", 5800);
    }

    // private void forceUpdateResult() {
    //     result = camera.getLatestResult();
    // }

    private PhotonPipelineResult getResult() {
        // if (result == null) {
        //     forceUpdateResult();
        // }
        // return result;
        return camera.getLatestResult();
    }

    @Override
    public double getLatency() {
        return result.getLatencyMillis() / 1000.0;
    }

    @Override
    public boolean hasTarget() {
        return getResult().hasTargets();
    }

    public double getDistance() {
        if (!getResult().hasTargets()) {
            System.out.printf("[WARNING]: getDistance() called with no targets");
            return 0.0;
        }
        return PhotonUtils.calculateDistanceToTargetMeters(
                Limelight.CAMERA_HEIGHT,
                Field.HUB_HEIGHT,
                Limelight.CAMERA_PITCH.toRadians(),
                Math.toRadians(getResult().getBestTarget().getPitch()));
    }

    public Angle getHorizontalOffset() {
        if (!getResult().hasTargets()) {
            System.out.printf("[WARNING]: getHorizontalOffset() called with no targets");
            return Angle.kZero;
        }
        return Angle.fromDegrees(getResult().getBestTarget().getYaw());
    }

    public Pose3d getPose3d() {
        Pose3d pose = ComputerVisionUtil.objectToRobotPose(
            // Apriltag Pose relative to robot's origin    
            Field.Tag1.POSE,
            result.getBestTarget().getBestCameraToTarget(),
            // transformation from robot's origin to camera
            new Transform3d(
                new Translation3d(Limelight.CAMERA_TO_CENTER, 0, Limelight.CAMERA_HEIGHT),
                new Rotation3d(0, Limelight.CAMERA_PITCH.toRadians(), 0)));
        return pose;
    }

    public Pose2d getPose2d() {
        Pose3d pose = getPose3d();
        return new Pose2d(pose.getTranslation().getX(), pose.getTranslation().getY(),
            pose.getRotation().toRotation2d());
    }

    @Override
    public int getTagID() {
        return getResult().getBestTarget().getFiducialId();
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Camera/Has Target", hasTarget());

        if (hasTarget()) {
            SmartDashboard.putNumber("Camera/Distance", getDistance());
            SmartDashboard.putNumber("Camera/Angle", getHorizontalOffset().toDegrees());

            Pose2d pose = getPose2d();
            SmartDashboard.putNumber("Camera/Pose X", pose.getX());
            SmartDashboard.putNumber("Camera/Pose Y", pose.getY());
            SmartDashboard.putNumber("Camera/Pose Angle", pose.getRotation().getDegrees());
        }

        if (DriverStation.isDisabled()) {
            camera.setLED(VisionLEDMode.kOff);
        } else {
            camera.setLED(VisionLEDMode.kOn);
        }
    }

}