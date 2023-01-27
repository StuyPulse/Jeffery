package com.stuypulse.robot.commands;

import com.stuypulse.robot.subsystems.SwerveDrive;
import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.angle.AngleController;
import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.streams.angles.filters.AFilter;
import com.stuypulse.stuylib.streams.angles.filters.AMotionProfile;
import com.stuypulse.stuylib.streams.filters.MotionProfile;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;

import com.stuypulse.robot.constants.Settings.AlignmentCommand.*;


public class SwerveDriveToPoseMP extends CommandBase {

    private SwerveDrive swerve;
    private Pose2d targetPose;

    //controllers
    private Controller xPID;
    private Controller yPID;
    private AngleController anglePID;

    public SwerveDriveToPoseMP(SwerveDrive swerve, Pose2d targetPose ){
        this.swerve = swerve;
        this.targetPose = targetPose;
        
        xPID = new PIDController(Translation.P, Translation.I, Translation.D);
            // .setSetpointFilter(new MotionProfile(3, 2));
        yPID = new PIDController(Translation.P, Translation.I, Translation.D);
            // .setSetpointFilter(new MotionProfile(3, 2));
        anglePID = new AnglePIDController(Rotation.P, Rotation.I, Rotation.D);
            // .setSetpointFilter(new AMotionProfile(5, 4));
    
        addRequirements(swerve);
    } 
    
    @Override
    public void execute(){
        Pose2d currentPose = swerve.getPose();
        
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(
            xPID.update(targetPose.getX(), currentPose.getX()),
            yPID.update(targetPose.getY(), currentPose.getY()),
            anglePID.update(Angle.fromRadians(targetPose.getRotation().getRadians()), Angle.fromRadians(currentPose.getRotation().getRadians()))
        );
        // log the angle
        
        final double epsilon = Units.inchesToMeters(4);
        final double epsilonDegrees = 5;

        if (xPID.isDone(epsilon)) chassisSpeeds.vxMetersPerSecond = 0;
        if (yPID.isDone(epsilon)) chassisSpeeds.vyMetersPerSecond = 0;
        if (anglePID.isDoneDegrees(epsilonDegrees))
            chassisSpeeds.omegaRadiansPerSecond = 0;

        swerve.setStates(chassisSpeeds, true);
    }
}
