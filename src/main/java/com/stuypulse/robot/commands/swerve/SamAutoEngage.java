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
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SamAutoEngage extends CommandBase {

    private final SwerveDrive swerve;

    private final Controller tiltController, velocityController;

    private final SmartAngle BALANCE_ANGLE;
    private final SmartNumber OFFSET, TARGET, CURRENT_POS, VELOCITY;
    
    public SamAutoEngage(SwerveDrive swerve) {
        this.swerve = swerve;

        tiltController = new PIDController(1,0,0);
        velocityController = new PIDController(1,0,0);

        BALANCE_ANGLE = new SmartAngle("Auto Engage/Balance Angle", Angle.kZero);

        OFFSET = new SmartNumber("Auto Engage/Offset", 0);
        TARGET = new SmartNumber("Auto Engage/Target", Field.CHARGING_STATION_CENTER.getX());
        CURRENT_POS = new SmartNumber("Auto Engage/Distance", swerve.getTranslation().getX());
        VELOCITY = new SmartNumber("Auto Engage/Velocity", 0);

        addRequirements(swerve);
    }

    @Override
    public void execute() {
        BALANCE_ANGLE.set(Angle.fromDegrees(Pitch.calculate(swerve.getGyroPitch(), swerve.getGyroRoll(), swerve.getAngle()).getDegrees()));
        OFFSET.set(tiltController.update(0, BALANCE_ANGLE.get().getRotation2d().getDegrees()));

        TARGET.add(OFFSET);
        CURRENT_POS.set(swerve.getTranslation().getX());
        VELOCITY.set(velocityController.update(TARGET.get(), CURRENT_POS.get()));

        swerve.setStates(new ChassisSpeeds(VELOCITY.get(), 0.0, 0.0), true);
    }

    @Override 
    public boolean isFinished() {
        return velocityController.isDone(0.1);
    }

    @Override 
    public void end(boolean interrupted) {
        swerve.setStates(new Vector2D(0, 0), 0);
    }
}
