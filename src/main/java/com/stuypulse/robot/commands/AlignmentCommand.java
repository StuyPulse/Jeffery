package com.stuypulse.robot.commands;

import com.stuypulse.robot.subsystems.SwerveDrive;
import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.streams.filters.MotionProfile;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class AlignmentCommand extends CommandBase {

    private SwerveDrive swerve;
    private Pose2d targetPose;

    //controllers
    private Controller xPID;
    private Controller yPID;
    private Controller anglePID;

    


    public AlignmentCommand(SwerveDrive swerve, Pose2d targetPose ){
        this.swerve = swerve;
        this.targetPose = targetPose;
        
        xPID = new PIDController(.1, .1, .1).
            setSetpointFilter(new MotionProfile(1, 3));
        yPID = new PIDController(.1, .1, .1).
            setSetpointFilter(new MotionProfile(1, 3));
        anglePID = new PIDController(.1, 0.1, 0.1).
            setSetpointFilter(new MotionProfile(1, 3));

    } 
    
    @Override
    public void execute(){
        Pose2d currentPose = swerve.getPose();
        
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(
            xPID.update(targetPose.getX(), currentPose.getX()),
            yPID.update(targetPose.getY(), currentPose.getY()),
            anglePID.update(targetPose.getRotation().getRadians(), currentPose.getRotation().getRadians())
        );

        swerve.setStates(chassisSpeeds, true);
    }
}
