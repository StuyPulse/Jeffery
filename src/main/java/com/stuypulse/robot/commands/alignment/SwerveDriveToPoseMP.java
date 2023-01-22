package com.stuypulse.robot.commands.alignment;

import com.stuypulse.robot.subsystems.SwerveDrive;
import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.angle.AngleController;
import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.streams.angles.filters.AFilter;
import com.stuypulse.stuylib.streams.booleans.BStream;
import com.stuypulse.stuylib.streams.booleans.filters.BDebounce;
import com.stuypulse.stuylib.streams.filters.MotionProfile;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;

import com.stuypulse.robot.constants.Settings.Swerve.*;
import com.stuypulse.robot.constants.Settings.AlignmentCommand.*;


public class SwerveDriveToPoseMP extends CommandBase {

    private SwerveDrive swerve;
    private Pose2d targetPose;

    //controllers
    private Controller xPID;
    private Controller yPID;
    private AngleController anglePID;
    private boolean continuousAlign;
    private BStream isAligned;

    public SwerveDriveToPoseMP(SwerveDrive swerve, Pose2d targetPose, boolean continuousAlign){
        this.swerve = swerve;
        this.targetPose = targetPose;
        this.continuousAlign = continuousAlign;
        
        xPID = new PIDController(Translation.P, Translation.I, Translation.D)
            .setSetpointFilter(new MotionProfile(Chassis.MAX_SPEED, Chassis.MAX_ACCELERATION));
        yPID = new PIDController(Translation.P, Translation.I, Translation.D)
            .setSetpointFilter(new MotionProfile(Chassis.MAX_SPEED, Chassis.MAX_ACCELERATION));
        anglePID = new AnglePIDController(Rotation.P, Rotation.I, Rotation.D)
            .setSetpointFilter(AFilter.create(x -> Angle.fromRadians(targetPose.getRotation().getRadians())));
        
        isAligned = BStream.create(() -> anglePID.isDoneDegrees(Aligned.ACCEPTABLE_ANGLE_ERROR))
            .and(() -> xPID.isDone(Aligned.ACCEPTABLE_DISPLACEMENT_ERROR))
            .and(() -> yPID.isDone(Aligned.ACCEPTABLE_DISPLACEMENT_ERROR))
            .filtered(new BDebounce.Rising(Aligned.DEBOUNCE_TIME));
        addRequirements(swerve);
    } 
    public SwerveDriveToPoseMP(SwerveDrive swerve, Pose2d targetPose) {
        this(swerve, targetPose, false);
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
        swerve.setStates(chassisSpeeds, true);
    }

    @Override
    public boolean isFinished(){
        if(continuousAlign){
            return false;
        } else{
            return isAligned.get();// how to check if aligned?
        }
    }
}
