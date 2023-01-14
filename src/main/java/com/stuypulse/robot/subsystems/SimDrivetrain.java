package com.stuypulse.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.stuypulse.robot.constants.Settings.Drivetrain.Encoders;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static com.stuypulse.robot.constants.Settings.Drivetrain.Motion.FeedForward.*;
import static com.stuypulse.robot.constants.Settings.Drivetrain.*;

public class SimDrivetrain extends IDrivetrain {

    public ICamera camera;

    private final DifferentialDrivetrainSim drivetrain;

    private final DifferentialDrivePoseEstimator poseEstimator;
    private final DifferentialDriveKinematics kinematics;

    private final Field2d field;

    public SimDrivetrain(ICamera camera) {
        this.camera = camera;

        drivetrain = new DifferentialDrivetrainSim(
                            LinearSystemId.identifyDrivetrainSystem(V, A, V, A),
                            DCMotor.getNEO(6),
                            7.73,
                            TRACK_WIDTH,
                            4,
                            VecBuilder.fill(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0));
        
        kinematics = new DifferentialDriveKinematics(TRACK_WIDTH);

        poseEstimator = new DifferentialDrivePoseEstimator(kinematics, drivetrain.getPose().getRotation(), 0, 0, 
            new Pose2d());
        poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.01, 0.1, Units.degreesToRadians(3)));

        field = new Field2d();

        SmartDashboard.putData("Edwin/Field", field);
    }

    public double getLeftDistance() {
        return 0;
    }

    @Override
    public Pose2d getPose2d() {
       
        return null;
    }

    @Override
    public Pose2d setPose2d() {
        // poseEstimator.update(drivetrain.getPose().getRotation(), , )
        return new Pose2d();
    }

    @Override
    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public void tankDriveVolts(double leftVolts, double rightVolts) {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void arcadeDrive(double speed, double angle) {
        // TODO Auto-generated method stub
        
    }
    
}
