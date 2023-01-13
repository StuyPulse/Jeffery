package com.stuypulse.robot.commands.swerve;

import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.subsystems.SwerveDrive;
import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.feedback.PIDController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SetpointAutoEngage extends CommandBase {
    
    private final SwerveDrive swerve;

    private final Controller controller;

    public SetpointAutoEngage() {
        this.swerve = SwerveDrive.getInstance();

        this.controller = new PIDController(0, 0, 0);

        addRequirements(swerve);
    }

    @Override
    public void execute() {
        
        // target Translation
        double xTarget = Units.inchesToMeters(Field.CHARGING_STATION_CENTER.getX());
        
        // distance from robot to target
        double distance = new Translation2d(xTarget, 0).minus(swerve.getTranslation()).getNorm();

        // get translation of swerve
        double translation = swerve.getTranslation().getX();

        // results in velocity 
        translation /= distance * MathUtil.clamp(
                // limits updated state of controller to within -0.5,0.5
                controller.update(xTarget, distance), -0.5, +0.5);

        ChassisSpeeds speeds = new ChassisSpeeds( 
            // set variable speeds to velocities (linear=Vx, sideways=0, angular = 0)
            translation,
            0,
            0
        );

        swerve.setStates(speeds); // set swerve drive to the speeds you want
    }

    @Override 
    public boolean isFinished() {
        return controller.isDone(0.1); // checks if the PID controller is done
    }

    @Override 
    public void end(boolean interrupted) {
        swerve.setStates(new ChassisSpeeds(0, 0, 0)); 
        // set the states(velocity) of the swerve drive to 0, which stops it
    }
}