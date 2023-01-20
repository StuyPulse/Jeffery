package com.stuypulse.robot.subsystems;

import java.util.Arrays;
import java.util.stream.Stream;

import com.kauailabs.navx.frc.AHRS;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Swerve.FrontLeft;
import com.stuypulse.robot.constants.Settings.Swerve.FrontRight;
import com.stuypulse.robot.constants.Settings.Swerve.BackLeft;
import com.stuypulse.robot.constants.Settings.Swerve.BackRight;
import com.stuypulse.robot.constants.Settings.Swerve.Chassis;
import com.stuypulse.robot.subsystems.modules.SL_SimModule;
import com.stuypulse.robot.subsystems.modules.SL_SwerveModule;
import com.stuypulse.robot.subsystems.modules.SwerveModule;
import com.stuypulse.robot.util.Pitch;
import com.stuypulse.stuylib.math.Vector2D;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;

public class SwerveDrive extends SubsystemBase {

    private static SwerveDrive instance = null;

    public static SwerveDrive getInstance(ICamera camera) {
        if (instance == null) {
            if (RobotBase.isReal()) {
                instance = new SwerveDrive(new SwerveModule[] {
                    new SL_SwerveModule(FrontRight.ID, FrontRight.MODULE_OFFSET, Ports.Swerve.FrontRight.TURN, Ports.Swerve.FrontRight.ENCODER, FrontRight.ABSOLUTE_OFFSET, Ports.Swerve.FrontRight.DRIVE),
                    new SL_SwerveModule(FrontLeft.ID, FrontLeft.MODULE_OFFSET, Ports.Swerve.FrontLeft.TURN, Ports.Swerve.FrontLeft.ENCODER, FrontLeft.ABSOLUTE_OFFSET, Ports.Swerve.FrontLeft.DRIVE),
                    new SL_SwerveModule(BackLeft.ID, BackLeft.MODULE_OFFSET, Ports.Swerve.BackLeft.TURN, Ports.Swerve.BackLeft.ENCODER, BackLeft.ABSOLUTE_OFFSET, Ports.Swerve.BackLeft.DRIVE),
                    new SL_SwerveModule(BackRight.ID, BackRight.MODULE_OFFSET, Ports.Swerve.BackRight.TURN, Ports.Swerve.BackRight.ENCODER, BackRight.ABSOLUTE_OFFSET, Ports.Swerve.BackRight.DRIVE)
                }, camera);
    
                // instance = new SwerveDrive(new SwerveModule[] {
                //     new SparkMax_Module(FrontRight.ID, FrontRight.MODULE_OFFSET, Ports.Swerve.FrontRight.TURN, Ports.Swerve.FrontRight.ENCODER, FrontRight.ABSOLUTE_OFFSET.getRotation2d(), Ports.Swerve.FrontRight.DRIVE),
                //     new SparkMax_Module(FrontLeft.ID, FrontLeft.MODULE_OFFSET, Ports.Swerve.FrontLeft.TURN, Ports.Swerve.FrontLeft.ENCODER, FrontLeft.ABSOLUTE_OFFSET.getRotation2d(), Ports.Swerve.FrontLeft.DRIVE),
                //     new SparkMax_Module(BackLeft.ID, BackLeft.MODULE_OFFSET, Ports.Swerve.BackLeft.TURN, Ports.Swerve.BackLeft.ENCODER, BackLeft.ABSOLUTE_OFFSET.getRotation2d(), Ports.Swerve.BackLeft.DRIVE),
                //     new SparkMax_Module(BackRight.ID, BackRight.MODULE_OFFSET, Ports.Swerve.BackRight.TURN, Ports.Swerve.BackRight.ENCODER, BackRight.ABSOLUTE_OFFSET.getRotation2d(), Ports.Swerve.BackRight.DRIVE)
                // });
            } else {
                instance = new SwerveDrive(new SwerveModule[] {
                    new SL_SimModule(FrontRight.ID, FrontRight.MODULE_OFFSET),
                    new SL_SimModule(FrontLeft.ID, FrontLeft.MODULE_OFFSET),
                    new SL_SimModule(BackLeft.ID, BackLeft.MODULE_OFFSET),
                    new SL_SimModule(BackRight.ID, BackRight.MODULE_OFFSET)
                }, camera);
            }
        }
        return instance;
    }


    /** MODULES **/
    private final SwerveModule[] modules;

    /** SENSORS **/
    private final AHRS gyro;

    /** ODOMETRY **/
    private final SwerveDriveKinematics kinematics;

    private final SwerveDriveOdometry odometry;
    private final SwerveDrivePoseEstimator poseEstimator;

    private final Field2d field;
    private final FieldObject2d[] module2ds;

    private final ICamera camera;


    public SwerveDrive(SwerveModule[] modules, ICamera camera) {
        this.modules = modules;

        gyro = new AHRS(SPI.Port.kMXP);

        kinematics = new SwerveDriveKinematics(
                getModuleStream()
                        .map(x -> x.getLocation())
                        .toArray(Translation2d[]::new));

        odometry = new SwerveDriveOdometry(kinematics, getGyroAngle(), getModulePositions());

        poseEstimator = new SwerveDrivePoseEstimator(kinematics, getGyroAngle(), getModulePositions(), new Pose2d());
        poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.01, 0.1, Units.degreesToRadians(3)));

        field = new Field2d();
        module2ds = new FieldObject2d[modules.length];
        for (int i = 0; i < modules.length; ++i) {
            module2ds[i] = field.getObject(modules[i].getId()+"-2d");
        }

        reset(new Pose2d());
        SmartDashboard.putData("Field", field);

        this.camera = camera;
    }

    public Field2d getField() {
        return field;
    }

    /** MODULE API **/

    public SwerveModule getModule(String id) {
        for (SwerveModule module : modules) {
            if (module.getId().equals(id))
                return module;
        }

        throw new IllegalArgumentException("Couldn't find module with ID \"" + id + "\"");
    }

    public ChassisSpeeds getChassisSpeeds() {
        return getKinematics().toChassisSpeeds(getModuleStates());
    }

    public Translation2d getVelocity() {
        var speeds = getChassisSpeeds();
        return new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
    }

    public Translation2d getTranslation() {
        return getPose().getTranslation();
    }

    public SwerveModule[] getModules() {
        return modules;
    }

    public Stream<SwerveModule> getModuleStream() {
        return Arrays.stream(getModules());
    }

    public SwerveModuleState[] getModuleStates() {
        return getModuleStream().map(x -> x.getState()).toArray(SwerveModuleState[]::new);
    }

    public SwerveModulePosition[] getModulePositions() {
        return getModuleStream().map(x -> x.getModulePosition()).toArray(SwerveModulePosition[]::new);
    }

    public void reset(Pose2d pose) {
        odometry.resetPosition(getGyroAngle(), getModulePositions(), pose);
        poseEstimator.resetPosition(getGyroAngle(), getModulePositions(), pose);
    }

    /** MODULE STATES API **/

    private static double getSaturation(SwerveModuleState[] states) {
        double sat = 1;
        for (var state : states) {
            sat = Math.max(sat, state.speedMetersPerSecond / Chassis.MAX_SPEED);
        }
        return sat;
    }

    public void setStates(Vector2D velocity, double omega, boolean fieldRelative) {
        if (fieldRelative) {
            final Rotation2d correction = new Rotation2d(0.5 * omega * Settings.DT);

            ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(velocity.y,
                    -velocity.x, -omega,
                    getAngle().plus(correction));

            for (int i = 0; i < 8; ++i) {
                double saturation = getSaturation(kinematics.toSwerveModuleStates(speeds));

                speeds = ChassisSpeeds.fromFieldRelativeSpeeds(velocity.y, -velocity.x,
                        -omega,
                        getAngle().plus(correction.times(1.0 / saturation)));
            }

            // setStatesRetainAngle(speeds);
            setStates(speeds, false);
        } else {
            setStates(new ChassisSpeeds(velocity.y, -velocity.x, -omega), false);
        }
    }

    private void setStatesRetainAngle(ChassisSpeeds robotSpeed) {
        var moduleStates = kinematics.toSwerveModuleStates(robotSpeed);
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, Chassis.MAX_SPEED);
        for (int i = 0; i < modules.length; ++i) {
            var currentState = modules[i].getState();
            if (Math.abs(moduleStates[i].speedMetersPerSecond) < Settings.Swerve.MIN_MODULE_VELOCITY) {
                modules[i].setTargetState(new SwerveModuleState(
                    moduleStates[i].speedMetersPerSecond, 
                    currentState.angle
                ));
            } else {
                modules[i].setTargetState(moduleStates[i]);
            }
        }
    }

    public void setStates(Vector2D velocity, double omega) {
        setStates(velocity, omega, true);
    }

    public void setStates(ChassisSpeeds robotSpeed, Boolean fieldRelative) {
        if (fieldRelative){
            robotSpeed = ChassisSpeeds.fromFieldRelativeSpeeds(robotSpeed, getAngle());
        }
        setStates(kinematics.toSwerveModuleStates(robotSpeed));
    }

    public void stop() {
        setStates(new ChassisSpeeds(0,0,0), false);
    }

    public void setStates(SwerveModuleState... states) {
        if (states.length != modules.length) {
            throw new IllegalArgumentException(
                    "Number of desired module states does not match number of modules (" + modules.length + ")");
        }

        SwerveDriveKinematics.desaturateWheelSpeeds(states, Chassis.MAX_SPEED);

        for (int i = 0; i < states.length; ++i) {
            modules[i].setTargetState(states[i]);
        }
    }

    /** GYRO API */

    public Rotation2d getGyroAngle() {
        return gyro.getRotation2d();
    }

    public Rotation2d getGyroPitch() {
        return Rotation2d.fromDegrees(gyro.getPitch());
    }

    public Rotation2d getGyroRoll() {
        return Rotation2d.fromDegrees(gyro.getRoll());
    }

    /** ODOMETRY API */

    private void updatePose() {
        poseEstimator.update(getGyroAngle(), getModulePositions());
        odometry.update(getGyroAngle(), getModulePositions());
        
        // if (false) {
        if (camera.hasTarget()) {
            Pose2d pose = camera.getPose2d();

            if (Math.abs(pose.getRotation().getDegrees() - getGyroAngle().getDegrees()) > 90) {
                pose = new Pose2d(pose.getTranslation(), getGyroAngle());
            }
            // poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(pose.getX(), pose.getY(), pose.getRotation().getRadians()));
            poseEstimator.addVisionMeasurement(
                pose, 
                Timer.getFPGATimestamp());

            // Translation2d robotPosition = new Translation2d(pose.getX(), pose.getY());
            // if (getPose().getTranslation().minus(robotPosition).getNorm() < 1.0) {
            //     poseEstimator.addVisionMeasurement(
            //             new Pose2d(pose.getX(), pose.getY(), pose.getRotation().toRotation2d()),
            //             Timer.getFPGATimestamp());
            // }
        }
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public Pose2d getOdometryPose() {
        return odometry.getPoseMeters();
    }

    public Rotation2d getAngle() {
        return getPose().getRotation();
    }

    public Rotation2d getOdometryAngle() {
        return odometry.getPoseMeters().getRotation();
    }

    public SwerveDriveKinematics getKinematics() {
        return kinematics;
    }

    @Override
    public void periodic() {
        updatePose();
        field.setRobotPose(getPose());

        var pose = getPose();
        for (int i = 0; i < modules.length; ++i) {
            module2ds[i].setPose(new Pose2d(
                pose.getTranslation().plus(modules[i].getLocation().rotateBy(getAngle())),
                modules[i].getState().angle.plus(getAngle())
            ));
        }

        // TODO: log angular velocity and velocity vector
        SmartDashboard.putNumber("Swerve/Pose Estimator X", getPose().getTranslation().getX());
        SmartDashboard.putNumber("Swerve/Pose Estimator Y", getPose().getTranslation().getY());
        SmartDashboard.putNumber("Swerve/Pose Estimator Angle (deg)", getAngle().getDegrees());

        SmartDashboard.putNumber("Swerve/Odometry X", getOdometryPose().getTranslation().getX());
        SmartDashboard.putNumber("Swerve/Odometry Y", getOdometryPose().getTranslation().getY());
        SmartDashboard.putNumber("Swerve/Odometry Angle (deg)", getOdometryAngle().getDegrees());
        // double deg = getAngle().getDegrees();
        // if (deg < 0) deg += 360;
        // SmartDashboard.putNumber("Swerve/0-360 Pose Angle (deg)", deg);

        SmartDashboard.putNumber("Swerve/Gyro Angle", getAngle().getDegrees());
        SmartDashboard.putNumber("Swerve/Gyro Pitch", getGyroPitch().getDegrees());
        SmartDashboard.putNumber("Swerve/Gyro Roll", getGyroRoll().getDegrees());

        SmartDashboard.putNumber("Swerve/Bottom Right Module Speed", getModule("Back Right").getState().speedMetersPerSecond);
        SmartDashboard.putNumber("Swerve/Pose Angle (rad)", getAngle().getRadians());

        SmartDashboard.putNumber("Swerve/Charge Station Angle (rad)", Pitch.calculate(
            getGyroPitch(),
            getGyroRoll(),
            getAngle()).getDegrees());

    }

    @Override
    public void simulationPeriodic() {
        // Integrate omega in simulation and store in gyro
        var speeds = getKinematics().toChassisSpeeds(getModuleStates());

        gyro.setAngleAdjustment(gyro.getAngle() - Math.toDegrees(speeds.omegaRadiansPerSecond * Settings.DT));
        // gyro.setAngleAdjustment(getPose().getRotation().getDegrees());
    }


}