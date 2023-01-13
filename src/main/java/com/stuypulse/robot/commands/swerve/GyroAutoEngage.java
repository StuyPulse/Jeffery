package com.stuypulse.robot.commands.swerve;

import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.subsystems.SwerveDrive;
import com.stuypulse.stuylib.math.Vector2D;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class GyroAutoEngage extends CommandBase {
    
    private final SwerveDrive swerve;

    public GyroAutoEngage() {
        this.swerve = SwerveDrive.getInstance();

        addRequirements(swerve);
    }

    @Override
    public void execute() {
        double target = Units.inchesToMeters(Field.CHARGING_STATION_CENTER.getX());

        double translation = swerve.getTranslation().getX() - target;
        double velocity = translation * (swerve.getGyroPitch() / 15);

        ChassisSpeeds speeds = new ChassisSpeeds(
            MathUtil.clamp(velocity, -0.5, +0.5),
            0,
            0
        );

        swerve.setStates(speeds);
    }

    @Override 
    public boolean isFinished() {
        return Math.abs(swerve.getGyroPitch()) < 2;
    }

    @Override 
    public void end(boolean interrupted) {
        swerve.setStates(new Vector2D(0, 0), 0.0);
    }
}