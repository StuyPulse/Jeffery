/************************ PROJECT DORCAS ************************/
/* Copyright (c) 2022 StuyPulse Robotics. All rights reserved.  */
/* This work is licensed under the terms of the MIT license.    */
/****************************************************************/

package com.stuypulse.robot.commands.drivetrain;

import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.angle.AngleController;
import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.robot.constants.Settings.Alignment;
import com.stuypulse.robot.subsystems.ICamera;
import com.stuypulse.robot.subsystems.camera.LLCamera;
import com.stuypulse.robot.subsystems.camera.LLCamera.AlignmentType;
import com.stuypulse.robot.subsystems.Drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class DrivetrainPipeAlignCommand extends CommandBase {

    private final Drivetrain drivetrain;
    private final LLCamera camera;

    protected final AngleController angleController;

    public DrivetrainPipeAlignCommand(Drivetrain drivetrain, LLCamera camera) {
        this.drivetrain = drivetrain;
        this.camera = camera;

        // handle errors
        this.angleController = Alignment.Angle.getPID();

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        drivetrain.setLowGear();

        camera.setAlignmentType(AlignmentType.REFLECTIVE_TAPE);
    }

    private double getTurn() {
        return angleController.update(camera.getHorizontalOffset(), Angle.kZero);
    }

    @Override
    public void execute() {
        drivetrain.arcadeDrive(0, getTurn());
    }

    @Override
    public void end(boolean interrupted) {
        camera.setAlignmentType(AlignmentType.APRIL_TAG);
    }
}