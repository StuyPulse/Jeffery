package com.stuypulse.robot.commands.swerve;

import com.stuypulse.robot.subsystems.SwerveDrive;
import com.stuypulse.robot.util.Pitch;
import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.network.SmartNumber;
import com.stuypulse.stuylib.streams.IStream;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class BasicGyroEngage extends CommandBase {
    
    private final SwerveDrive swerve;
    
    private Controller control;

    private SmartNumber MAX_TILT, MAX_ENGAGE_SPEED;

    public BasicGyroEngage(SwerveDrive swerve) {
        this.swerve = swerve;

        MAX_TILT = new SmartNumber("Engage/Max Tilt (deg)", 20.0);
        MAX_ENGAGE_SPEED = new SmartNumber("Engage/Max Engage Speed (m per s)", 1.0);

        control = new PIDController(IStream.create(() -> MAX_ENGAGE_SPEED.get() / MAX_TILT.get()).number(), 0, null);
    }

    @Override
    public void execute() {
        double speed = control.update(
            0,
            Pitch.calculate(swerve.getGyroPitch().getRadians(), swerve.getGyroRoll().getRadians(), swerve.getAngle().getRadians()));
        
        swerve.setStates(new ChassisSpeeds(speed, 0, 0), true);
    }

    @Override
    public void end(boolean interrupted) {
        swerve.setStates(new ChassisSpeeds(), true);
    }

}
