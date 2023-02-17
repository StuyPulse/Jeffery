package com.stuypulse.robot.subsystems.camera;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.ICamera;
import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.network.limelight.Limelight;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/* todo: make ICamera, SimCamera */
public class LLCamera extends ICamera {

	private Limelight limelight;
	
	public LLCamera() {
		limelight = Limelight.getInstance(); 

		for (int port : Settings.Limelight.PORTS) {
            PortForwarder.add(port, "limelight.local", port);
        }
        CameraServer.startAutomaticCapture();
	}

	public double getLatency() {
		return limelight.getLatencyMs() / 1000.0;
	}

	public Angle getHorizontalOffset() {
		if (!hasTarget()) {
            // System.out.println("Unable To Find Target! [getHorizontal() was called]");
            return Angle.kZero;
        }
		double txDegrees = limelight.getTargetXAngle();

		Angle txAngle = Angle.fromDegrees(txDegrees);
		
		return txAngle;
	}

	public Angle getVerticalOffset() {
		if (!hasTarget()) {
            // System.out.println("Unable To Find Target! [getVerticalOffset() was called]");
            return Angle.kZero;
        }
		double tyDegrees = limelight.getTargetYAngle();
		
		Angle tyAngle = Angle.fromDegrees(tyDegrees);

		return tyAngle;
	}

	public double getDistance() {
		Angle ty = getVerticalOffset().add(Settings.Limelight.CAMERA_PITCH);

		double distance = (Settings.Field.HUB_HEIGHT-Settings.Limelight.CAMERA_HEIGHT) / ty.tan();

		return distance + Settings.Field.HUB_TO_CENTER + Settings.Limelight.CAMERA_TO_CENTER;
	}

	
    public boolean hasTarget() {
        return limelight.getValidTarget();
    }

	public Pose2d getPose2d() {
		Pose3d pose = getPose3d();
		return new Pose2d(pose.getTranslation().getX(), pose.getTranslation().getY(), pose.getRotation().toRotation2d());
	}

	public Pose3d getPose3d() {
		if (limelight.getRobotPose() == null) {
			return new Pose3d();
		}
        return limelight.getRobotPose();
	}

	@Override
	public void periodic() {
		if (!limelight.isConnected()) {
			// System.out.println("[WARNING] Limelight is disconnected.");
		}

		SmartDashboard.putNumber("Camera/Distance", getDistance());
		SmartDashboard.putNumber("Camera/Angle", getHorizontalOffset().toDegrees());

		Pose2d pose = getPose2d();

		SmartDashboard.putNumber("Camera/Pose X", pose.getX());
		SmartDashboard.putNumber("Camera/Pose Y", pose.getY());
		SmartDashboard.putNumber("Camera/Pose Rotation", pose.getRotation().getDegrees());
			
	}

}