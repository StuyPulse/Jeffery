package com.stuypulse.robot.commands.alignment;

import com.stuypulse.robot.subsystems.SwerveDrive;
import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.angle.AngleController;
import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.streams.booleans.BStream;
import com.stuypulse.stuylib.streams.booleans.filters.BDebounce;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;

import com.stuypulse.robot.constants.Settings.AlignmentCommand.*;


public class SwerveDriveToPose extends CommandBase {

    private SwerveDrive swerve;
    private Pose2d targetPose;

    //controllers
    private Controller xPID;
    private Controller yPID;
    private AngleController anglePID;

    private boolean continuousAlign;
    private BStream isAligned;

    public SwerveDriveToPose(SwerveDrive swerve, Pose2d targetPose, boolean continuousAlign){
        this.swerve = swerve;
        this.targetPose = targetPose;
        this.continuousAlign = continuousAlign;
        
        xPID = new PIDController(Translation.P, Translation.I, Translation.D);
        yPID = new PIDController(Translation.P, Translation.I, Translation.D);
        anglePID = new AnglePIDController(Rotation.P, Rotation.I, Rotation.D);

        isAligned = BStream.create(() -> anglePID.isDoneDegrees(Aligned.ACCEPTABLE_ANGLE_ERROR))
                .and(() -> xPID.isDone(Aligned.ACCEPTABLE_DISPLACEMENT_ERROR))
                .and(() -> yPID.isDone(Aligned.ACCEPTABLE_DISPLACEMENT_ERROR))
                .filtered(new BDebounce.Rising(Aligned.DEBOUNCE_TIME));
        addRequirements(swerve);  
    } 

    public SwerveDriveToPose(SwerveDrive swerve, Pose2d targetPose){
        this(swerve, targetPose, false);
    }
    public boolean translationMatch(){
        return ((swerve.getTranslation().getX() - targetPose.getX() < 0.3) 
              && swerve.getTranslation().getY() - targetPose.getY() < 0.3);
    }

    @Override
    public void execute(){
        Pose2d currentPose = swerve.getPose();
        
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(
            xPID.update(targetPose.getX(), currentPose.getX()),
            yPID.update(targetPose.getY(), currentPose.getY()),
            anglePID.update(Angle.fromRadians(targetPose.getRotation().getRadians()), Angle.fromRadians(currentPose.getRotation().getRadians()))
        );

        swerve.setStates(chassisSpeeds, true);
    }

    @Override
    public boolean isFinished(){
        if(continuousAlign){
            return false;
        } else{
            return isAligned.get();// how to check if aligned?
        }
        // if aligned for 0.3 seconds, score
    }
}
