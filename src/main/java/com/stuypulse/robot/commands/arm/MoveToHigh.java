package com.stuypulse.robot.commands.arm;

import com.stuypulse.robot.constants.Ports.Gamepad;
import com.stuypulse.robot.subsystems.IArm;
import com.stuypulse.robot.subsystems.SimArm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class MoveToHigh extends InstantCommand {
    public final IArm arm;

    public MoveToHigh(IArm arm) {
        this.arm = arm;
        addRequirements(arm);

    }

    public void initialize() {
        arm.setTargetShoulderAngle(-20);
        arm.setTargetWristAngle(20);
    }
}
