package com.stuypulse.robot.subsystems.camera;

import java.util.Optional;

import com.stuypulse.robot.subsystems.ICamera;
import com.stuypulse.robot.util.AprilTagData;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArrayEntry;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.IntegerEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LLCamera extends ICamera {
    
    private static double kCaptureDelayMs = 11.0;

    private String tableName;

    private final DoubleArrayEntry botposeEntry;
    private final DoubleEntry latencyEntry;
    private final IntegerEntry idEntry;

    public LLCamera() {
        this.tableName = "limelight-back";

        NetworkTable limelight = NetworkTableInstance.getDefault().getTable(tableName);

        latencyEntry = limelight.getDoubleTopic("tl").getEntry(0);
        botposeEntry = limelight.getDoubleArrayTopic("botpose").getEntry(new double[] {});
        idEntry = limelight.getIntegerTopic("tid").getEntry(0);
    }

    public String getTableName() {
        return tableName;
    }

    @Override
    public Optional<AprilTagData> getPoseData() {
        double[] botposeData = botposeEntry.get();
        
        if (botposeData.length != 6) {
            return Optional.empty();
        }

        Pose2d botpose = new Pose2d(botposeData[0], botposeData[1], Rotation2d.fromDegrees(botposeData[5]));
        double latency = Units.millisecondsToSeconds(latencyEntry.get() + kCaptureDelayMs);
        int id = (int) idEntry.get();
        
        return Optional.of(new AprilTagData(botpose, latency, id));
    }

}