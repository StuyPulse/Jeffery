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

    public SetpointAutoEngage(SwerveDrive swerve) {
        this.swerve = swerve;

        this.controller = new PIDController(1, 0, 0);

        addRequirements(swerve);
    }

    @Override
    public void execute() {
        
        // target Translation
        double target = Units.inchesToMeters(Field.CHARGING_STATION_CENTER.getX());
        
        // distance from robot to target
        double distance = swerve.getTranslation().getX();

        // results in velocity 
        double velocity = controller.update(target, distance);

        swerve.setStates(
            new ChassisSpeeds(velocity, 0, 0),
            true
        ); // set swerve drive to the speeds you want
    }

    @Override 
    public boolean isFinished() {
        return controller.isDone(0.1); // checks if the PID controller is done
    }

    @Override 
    public void end(boolean interrupted) {
        swerve.stop();
        // set the states(velocity) of the swerve drive to 0, which stops it
    }
}