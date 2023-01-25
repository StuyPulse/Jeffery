package com.stuypulse.robot.commands.swerve;

import com.stuypulse.robot.subsystems.SwerveDrive;
import com.stuypulse.robot.util.Pitch;
import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.network.SmartNumber;
import com.stuypulse.stuylib.network.SmartString;
import com.stuypulse.stuylib.streams.IStream;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class BasicGyroEngage extends CommandBase {
    
    private final SwerveDrive swerve;
    
    private Controller control;

    private SmartNumber MAX_TILT, MAX_ENGAGE_SPEED;

    public BasicGyroEngage(SwerveDrive swerve) {
        this.swerve = swerve;

        MAX_TILT = new SmartNumber("Auto Engage/Max Tilt", 15.0);
        MAX_ENGAGE_SPEED = new SmartNumber("Auto Engage/Max Engage Velocity", 0.4);

        control = new PIDController(IStream.create(() -> MAX_ENGAGE_SPEED.get() / MAX_TILT.get()).number(), 0, 0);
    }

    @Override
    public void execute() {
        double balanceAngle = -Pitch.calculate(swerve.getGyroPitch(), swerve.getGyroRoll(), swerve.getAngle()).getDegrees();
        double speed = control.update(0, balanceAngle);
        
        swerve.setStates(new ChassisSpeeds(speed, 0, 0), true);

        SmartDashboard.putNumber("Auto Engage/Balance Angle", balanceAngle);
        SmartDashboard.putNumber("Auto Engage/Velocity", speed);
    }

    @Override
    public void end(boolean interrupted) {
        swerve.setStates(new ChassisSpeeds(), true);
    }

}
