package com.stuypulse.robot.commands.arm;

import com.stuypulse.robot.constants.Ports.Gamepad;
import com.stuypulse.robot.subsystems.IArm;
import com.stuypulse.robot.subsystems.SimArm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class Holding extends InstantCommand {
    public final IArm arm;

    public Holding(IArm arm) {
        this.arm = arm;
        addRequirements(arm);

    }

    public void initialize() {
        arm.setTargetShoulderAngle(-40);
        arm.setTargetWristAngle(100);
    }
}
