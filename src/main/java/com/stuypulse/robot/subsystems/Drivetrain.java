package com.stuypulse.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import static com.stuypulse.robot.constants.Ports.Drivetrain.*;
import static com.stuypulse.robot.constants.Settings.Drivetrain.*;

import static com.stuypulse.robot.constants.Motors.Drivetrain.*;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {

    ICamera camera;

    // Enum used to store the state of the gear
    public static enum Gear {
        HIGH,
        LOW
    };

    private final CANSparkMax[] left;
    private final CANSparkMax[] right;

    private final RelativeEncoder leftEncoder;
    private final RelativeEncoder rightEncoder;

    // private final Controller leftController, rightController;
    // private SmartNumber leftTargetSpeed, rightTargetSpeed;

    private final DifferentialDrive drivetrain;

    private Gear gear;
    private final Solenoid gearShift;

    private final AHRS navx;

    // private final DifferentialDriveOdometry odometry;
    private final DifferentialDrivePoseEstimator poseEstimator;
    private final DifferentialDriveKinematics kinematics;

    private final Field2d field;

    public Drivetrain(ICamera camera) {
        this.camera = camera;
        left = new CANSparkMax[] {
                new CANSparkMax(LEFT_TOP, MotorType.kBrushless),
                new CANSparkMax(LEFT_BOTTOM, MotorType.kBrushless)
        };

        right = new CANSparkMax[] {
                new CANSparkMax(RIGHT_TOP, MotorType.kBrushless),
                new CANSparkMax(RIGHT_BOTTOM, MotorType.kBrushless)
        };

        leftEncoder = left[0].getEncoder();
        rightEncoder = right[0].getEncoder();

        configMotors();

        // leftController = new Feedforward.Drivetrain(Motion.Feedforward.kS,
        // Motion.Feedforward.kV, Motion.Feedforward.kA).velocity()
        // .add(new PIDController(Motion.PID.kP, Motion.PID.kI, Motion.PID.kD));

        // rightController = new Feedforward.Drivetrain(Motion.Feedforward.kS,
        // Motion.Feedforward.kV, Motion.Feedforward.kA).velocity()
        // .add(new PIDController(Motion.PID.kP, Motion.PID.kI, Motion.PID.kD));

        // leftTargetSpeed = new SmartNumber("Edwin/Left Target Speed", 0);
        // rightTargetSpeed = new SmartNumber("Edwin/Right TargetSpeed", 0);

        drivetrain = new DifferentialDrive(new MotorControllerGroup(left), new MotorControllerGroup(right));

        gear = Gear.HIGH;
        gearShift = new Solenoid(PneumaticsModuleType.CTREPCM, GEAR_SHIFT);
        gearShift.set(true);

        navx = new AHRS(SPI.Port.kMXP);
        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);

        kinematics = new DifferentialDriveKinematics(TRACK_WIDTH);

        // odometry = new DifferentialDriveOdometry(getRotation2d(), 0, 0);

        poseEstimator = new DifferentialDrivePoseEstimator(getKinematics(), getRotation2d(), 0, 0, getPose());
        poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.01, 0.1, Units.degreesToRadians(3)));

        setPose(new Pose2d(3.302, 0, new Rotation2d()));

        field = new Field2d();
        SmartDashboard.putData("Edwin/Field", field);
    }

    private void configMotors() {

        leftEncoder.setVelocityConversionFactor(Encoders.HIGH_GEAR_DISTANCE_PER_ROTATION / 60);
        leftEncoder.setPositionConversionFactor(Encoders.HIGH_GEAR_DISTANCE_PER_ROTATION);
        rightEncoder.setVelocityConversionFactor(Encoders.HIGH_GEAR_DISTANCE_PER_ROTATION / 60);
        rightEncoder.setPositionConversionFactor(Encoders.HIGH_GEAR_DISTANCE_PER_ROTATION);

        for (CANSparkMax motor : left) {
            LEFT.configure(motor);
        }

        for (CANSparkMax motor : right) {
            RIGHT.configure(motor);
        }
    }

    /*********************
     * ENCODER FUNCTIONS *
     *********************/

    // Set the distance traveled in one rotation of the motor
    public void setNEODistancePerRotation(double distance) {
        leftEncoder.setPositionConversionFactor(distance);
        rightEncoder.setPositionConversionFactor(distance);
    }

    // Distance
    private double getLeftDistance() {
        return leftEncoder.getPosition();
    }

    private double getRightDistance() {
        return rightEncoder.getPosition();
    }

    public double getDistance() {
        return (getLeftDistance() + getRightDistance()) / 2.0;
    }

    // Velocity
    private double getLeftVelocity() {
        return leftEncoder.getVelocity();
    }

    private double getRightVelocity() {
        return rightEncoder.getVelocity();
    }

    public double getVelocity() {
        return (getLeftVelocity() + getRightVelocity()) / 2.0;
    }

    /***************
     * ROBOT ANGLE *
     ***************/

    // Gets current Angle of the Robot as a Rotation2d (contiuous / not +-180)
    public Rotation2d getRotation2d() {
        return navx.getRotation2d();
    }

    /**********************
     * ODOMETRY FUNCTIONS *
     **********************/

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public void setPose(Pose2d pose) {
        leftEncoder.setPosition(0.);
        rightEncoder.setPosition(0.);

        // odometry.resetPosition(getRotation2d(), 0, 0, pose);

        poseEstimator.resetPosition(getRotation2d(), 0, 0, pose);
    }

    private void resetNavX() {
        navx.reset();
    }

    public double getGyroAngle() {
        return navx.getAngle();
    }

    public DifferentialDriveKinematics getKinematics() {
        return kinematics;
    }

    // public TrajectoryConfig getTrajectoryConfig() {
    //     return new TrajectoryConfig(
    //             Motion.MAX_VELOCITY,
    //             Motion.MAX_ACCELERATION).setKinematics(kinematics);
    // }

    /********************
     * DRIVING COMMANDS *
     ********************/

    public void stop() {
        drivetrain.stopMotor();
        tankDriveVolts(0, 0);
    }

    public void tankDrive(double leftMetersPerSecond, double rightMetersPerSecond) {
        drivetrain.tankDrive(leftMetersPerSecond, rightMetersPerSecond);
    }

    public void arcadeDrive(double speed, double angle) {
        drivetrain.arcadeDrive(speed, angle);
    }


    public void tankDriveVolts(double leftVolts, double rightVolts) {
        for (MotorController motor : left) {
            motor.setVoltage(leftVolts);
        }

        for (MotorController motor : right) {
            motor.setVoltage(rightVolts);
        }

        drivetrain.feed();
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(getLeftVelocity(), getRightVelocity());
    }

    /***************
     * GEAR SHIFT *
     ***************/

    // Gets the current gear the robot is in
    public Gear getGear() {
        return gear;
    }

    // Sets the current gear the robot is in
    public void setGear(Gear gear) {
        if (this.gear != gear) {
            this.gear = gear;
            if (this.gear == Gear.HIGH) {
                gearShift.set(true);
                setNEODistancePerRotation(
                        Encoders.HIGH_GEAR_DISTANCE_PER_ROTATION);
                reset();
            } else {
                gearShift.set(false);
                setNEODistancePerRotation(
                        Encoders.LOW_GEAR_DISTANCE_PER_ROTATION);
                reset();
            }
        }
    }

    // Sets robot into low gear
    public void setLowGear() {
        setGear(Gear.LOW);
    }

    // Sets robot into high gear
    public void setHighGear() {
        setGear(Gear.HIGH);
    }

    public void reset(Pose2d location) {
        resetNavX();
        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);
        // odometry.resetPosition(
        //         location.getRotation(),
        //         leftEncoder.getPosition(),
        //         rightEncoder.getPosition(),
        //         getPose());

        poseEstimator.resetPosition(
                location.getRotation(),
                leftEncoder.getPosition(),
                rightEncoder.getPosition(),
                getPose());
    }

    public void reset() {
        reset(getPose());
    }

    @Override
    public void periodic() {
        // odometry.update(getRotation2d(), getLeftDistance(), getRightDistance());
        poseEstimator.update(getRotation2d(), getLeftDistance(), getRightDistance());
        field.setRobotPose(getPose());

        // LOGGING

        SmartDashboard.putNumber("Edwin/Left Distance", getLeftDistance());
        SmartDashboard.putNumber("Edwin/Right Distance", getRightDistance());

        SmartDashboard.putNumber("Edwin/Left Vel", getLeftVelocity());
        SmartDashboard.putNumber("Edwin/Right Vel", getRightVelocity());

        SmartDashboard.putNumber("Edwin/Odometry Pose X", getPose().getX());
        SmartDashboard.putNumber("Edwin/Odometry Pose Y", getPose().getY());
        SmartDashboard.putNumber("Edwin/Odometry Gyro Angle", getRotation2d().getDegrees());

        Pose2d estimatedPose = poseEstimator.getEstimatedPosition();

        SmartDashboard.putNumber("Edwin/Estimater Pose X", estimatedPose.getX());
        SmartDashboard.putNumber("Edwin/Estimater Pose Y", estimatedPose.getY());
        SmartDashboard.putNumber("Edwin/Estimater Pose Angle", estimatedPose.getRotation().getDegrees());

        if (camera.hasTarget()) {
            Pose3d pose = camera.getPose3d();
            Translation2d robotPosition = new Translation2d(pose.getX(), pose.getY());
            if (getPose().getTranslation().minus(robotPosition).getNorm() < 1.0) {
                poseEstimator.addVisionMeasurement(
                        new Pose2d(pose.getX(), pose.getY(), pose.getRotation().toRotation2d()),
                        Timer.getFPGATimestamp());
            }
        }
    }

    public Field2d getField2d() {
        return field;
    }
}