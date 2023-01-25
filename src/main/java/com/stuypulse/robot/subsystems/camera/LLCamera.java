package com.stuypulse.robot.subsystems.camera;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.ICamera;
import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.network.limelight.Limelight;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.networktables.NetworkTableInstance;
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

	@Override
	public double getLatency() {
		return limelight.getLatencyMs() / 1000.0;
	}

	public int getTagID() {
		// return Limelight.getTagID(); 
		return (int)NetworkTableInstance.getDefault().getTable("limelight").getIntegerTopic("tid").getEntry(-1).get();
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
			System.out.println("[WARNING] Limelight is disconnected.");
		}

		Pose2d pose = getPose2d();

		SmartDashboard.putNumber("Camera/Pose X", pose.getX());
		SmartDashboard.putNumber("Camera/Pose Y", pose.getY());
		SmartDashboard.putNumber("Camera/Pose Rotation", pose.getRotation().getDegrees());
	}

}