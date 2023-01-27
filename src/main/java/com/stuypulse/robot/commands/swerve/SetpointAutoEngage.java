package com.stuypulse.robot.commands.swerve;

import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.subsystems.SwerveDrive;
import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.network.SmartNumber;
import com.stuypulse.stuylib.network.SmartString;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SetpointAutoEngage extends CommandBase {
    
    private final SwerveDrive swerve;

    private final Controller controller;

    private final SmartNumber TARGET, CURRENT_POS, VELOCITY;

    public SetpointAutoEngage(SwerveDrive swerve) {
        this.swerve = swerve;

        this.controller = new PIDController(1, 0, 0);

        TARGET = new SmartNumber("Auto Engage/Target", Field.CHARGING_STATION_CENTER.getX());
        CURRENT_POS = new SmartNumber("Auto Engage/Distance", swerve.getTranslation().getX());
        VELOCITY = new SmartNumber("Auto Engage/Velocity", 0);

        addRequirements(swerve);
    }

    @Override
    public void execute() {
        CURRENT_POS.set(swerve.getTranslation().getX());
        VELOCITY.set(controller.update(TARGET.get(), CURRENT_POS.get()));
        swerve.setStates(new ChassisSpeeds(VELOCITY.get(), 0, 0), true);
    }

    @Override 
    public boolean isFinished() {
        return controller.isDone(0.1);
    }

    @Override 
    public void end(boolean interrupted) {
        swerve.stop();
    }
}