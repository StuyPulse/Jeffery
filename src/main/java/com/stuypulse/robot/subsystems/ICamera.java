package com.stuypulse.robot.subsystems;

import com.stuypulse.robot.subsystems.camera.*;
// import com.stuypulse.robot.subsystems.camera.SimCamera;
import com.stuypulse.stuylib.math.Angle;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class ICamera extends SubsystemBase {

    private static ICamera instance = null;

    public static ICamera getInstance() {
        if (instance == null) {
            instance = RobotBase.isReal() ? new LLCamera() : new LLCamera();
        }
        return instance;
    }

    public abstract boolean hasRobotPose();

    public abstract Pose2d getRobotPose();

    public abstract double getLatency();

    // public abstract int getTagID();
}