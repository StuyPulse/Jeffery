package com.stuypulse.robot.commands.alignment;

import com.stuypulse.robot.subsystems.SwerveDrive;
import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.angle.AngleController;
import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.math.Angle;

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

    public SwerveDriveToPose(SwerveDrive swerve, Pose2d targetPose ){
        this.swerve = swerve;
        this.targetPose = targetPose;
        
        xPID = new PIDController(Translation.P, Translation.I, Translation.D);
        yPID = new PIDController(Translation.P, Translation.I, Translation.D);
        anglePID = new AnglePIDController(Rotation.P, Rotation.I, Rotation.D);

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
        swerve.setStates(chassisSpeeds, true);
    }
}
