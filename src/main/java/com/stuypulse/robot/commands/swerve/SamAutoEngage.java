package com.stuypulse.robot.commands.swerve;

import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.subsystems.SwerveDrive;
import com.stuypulse.robot.util.Pitch;
import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.math.Vector2D;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SamAutoEngage extends CommandBase {

    private final SwerveDrive swerve;

    private final Controller tiltController, velocityController;
    
    public SamAutoEngage(SwerveDrive swerve) {
        this.swerve = swerve;

        tiltController = new PIDController(1,0,0);
        velocityController = new PIDController(1,0,0);

        addRequirements(swerve);
    }

    @Override
    public void execute() {
        double balanceAngle = Pitch.calculate(swerve.getGyroPitch(), swerve.getGyroRoll(), swerve.getAngle().getDegrees());
        double offset = tiltController.update(0, balanceAngle);

        double target = Units.inchesToMeters(Field.CHARGING_STATION_CENTER.getX());
        target += offset;

        double distance = swerve.getTranslation().getX();

        double velocity = velocityController.update(target, distance);

        swerve.setStates(new ChassisSpeeds(velocity, 0.0, 0.0), true);
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
