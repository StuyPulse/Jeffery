package com.stuypulse.robot.commands.drivetrain;

import static com.stuypulse.robot.constants.Settings.Driver.*;
import com.stuypulse.robot.subsystems.Drivetrain;
import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.math.SLMath;
import com.stuypulse.stuylib.streams.IStream;
import com.stuypulse.stuylib.streams.filters.LowPassFilter;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class DrivetrainDriveCommand extends CommandBase {
    private final Drivetrain drivetrain;

    private final IStream speed;
    private final IStream angle;

    public DrivetrainDriveCommand(Drivetrain drivetrain, Gamepad gamepad) {
        this.drivetrain = drivetrain;

        this.speed = IStream.create(() -> gamepad.getRightTrigger() - gamepad.getLeftTrigger())
                .filtered(
                        x -> SLMath.deadband(x, SPEED_DEADBAND.get()),
                        x -> SLMath.spow(x, SPEED_POWER.get()),
                        new LowPassFilter(SPEED_FILTER));

        this.angle = IStream.create(() -> gamepad.getLeftX())
                .filtered(
                        x -> SLMath.deadband(x, ANGLE_DEADBAND.get()),
                        x -> SLMath.spow(x, ANGLE_POWER.get()),
                        new LowPassFilter(ANGLE_FILTER));

        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        drivetrain.arcadeDrive(speed.get(), angle.get());
    }
}
