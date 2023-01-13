// package com.stuypulse.robot.subsystems.camera;

// import com.stuypulse.robot.constants.Settings.Field;
// import com.stuypulse.robot.subsystems.ICamera;
// import com.stuypulse.robot.subsystems.SwerveDrive;
// import com.stuypulse.stuylib.math.Angle;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// public class SimCamera extends ICamera {

//     private static final double MAX_FOV = 27.0;

//     private final SwerveDrive swerve;

//     public SimCamera(SwerveDrive swerve) {
//         this.swerve = swerve;

//         swerve.getField().getObject("Hub").setPose(new Pose2d(Field.HUB, new Rotation2d()));
//     }

//     @Override
//     public double getLatency() {
//         return 0.0;
//     }

//     @Override
//     public double getDistance() {
//         if (!hasTarget()) {
//             return 0.0;
//         }
//         return 0;
//     }

//     private Angle getRawHorizontalOffset() {
//         return Angle.fromRotation2d(
//             swerve.getPose().getTranslation().minus(Field.HUB)
//                 .getAngle().minus(swerve.getGyroAngle()));
//     }

//     @Override
//     public Angle getHorizontalOffset() {
//         if (hasTarget())
//             return getRawHorizontalOffset();
        
//         return Angle.kZero;
//     }

//     @Override
//     public boolean hasTarget() {
//         return Math.abs(getRawHorizontalOffset().toDegrees()) <= MAX_FOV;
//     }

//     @Override
//     public Pose2d getRobotPose() {
//         return swerve.getPose();
//     } 

//     @Override
//     public void simulationPeriodic() {
//         SmartDashboard.putBoolean("Camera/Has Target", hasTarget());
//         SmartDashboard.putNumber("Camera/Raw Horiz Offset (deg)", getRawHorizontalOffset().toDegrees());
//     }
    
// }