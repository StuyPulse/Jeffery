package com.stuypulse.robot.commands.swerve;

import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.subsystems.SwerveDrive;
import com.stuypulse.robot.util.Pitch;
import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.math.Vector2D;
import com.stuypulse.stuylib.network.SmartAngle;
import com.stuypulse.stuylib.network.SmartNumber;
import com.stuypulse.stuylib.network.SmartString;
import com.stuypulse.stuylib.streams.IStream;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SamAutoEngage extends CommandBase {

    private final SwerveDrive swerve;

    private final Controller tiltController, velocityController;
    
    public SamAutoEngage(SwerveDrive swerve) {
        this.swerve = swerve;

        tiltController = new PIDController(6.0/15.0, 0,0);
        velocityController = new PIDController(1,0,0);

        addRequirements(swerve);
    }

    @Override
    public void execute() {
        var target = Field.CHARGING_STATION_CENTER.getX();
        var balanceAngle = Pitch.calculate(swerve.getGyroPitch(), swerve.getGyroRoll(), swerve.getAngle()).getDegrees();
        var offset = tiltController.update(0, balanceAngle);

        target += offset;
        var position = swerve.getTranslation().getX();
        var velocity = velocityController.update(target, position);

        swerve.setStates(new ChassisSpeeds(velocity, 0.0, 0.0), true);
        
        SmartDashboard.putNumber("Auto Engage/position", position);
        SmartDashboard.putNumber("Auto Engage/vel", velocity);
    }

    @Override 
    public boolean isFinished() {
        // return velocityController.isDone(0.1);
        return false;
    }

    @Override 
    public void end(boolean interrupted) {
        swerve.setStates(new Vector2D(0, 0), 0);
    }
}
