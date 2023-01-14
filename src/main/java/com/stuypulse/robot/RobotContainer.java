/************************ PROJECT PHIL ************************/
/* Copyright (c) 2023 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot;

import com.stuypulse.robot.commands.SwerveDriveToPose;
import com.stuypulse.robot.commands.SwerveDriveToPoseMP;
import com.stuypulse.robot.commands.auton.DoNothingAuton;
import com.stuypulse.robot.commands.swerve.SwerveDriveDrive;
import com.stuypulse.robot.commands.swerve.SwerveHome;
import com.stuypulse.robot.commands.swerve.SwerveDriveHome;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.subsystems.SwerveDrive;
import com.stuypulse.robot.util.BootlegXbox;
import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.input.gamepads.AutoGamepad;
import com.stuypulse.stuylib.input.gamepads.Xbox;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class RobotContainer {

    // Gamepads
    public final Gamepad driver = new Xbox(Ports.Gamepad.DRIVER);
    public final Gamepad operator = new Xbox(Ports.Gamepad.OPERATOR);

    // Subsystem
    public final SwerveDrive swerve = SwerveDrive.getInstance();

    // Autons
    private static SendableChooser<Command> autonChooser = new SendableChooser<>();

    // Robot container

    public RobotContainer() {
        configureDefaultCommands();
        configureButtonBindings();
        configureAutons();

        DriverStation.silenceJoystickConnectionWarning(true);
    }

    /****************/
    /*** DEFAULTS ***/
    /****************/

    private void configureDefaultCommands() {
        swerve.setDefaultCommand(new SwerveDriveDrive(swerve, driver));
    }

    /***************/
    /*** BUTTONS ***/
    /***************/

    private void configureButtonBindings() {

        driver.getRightButton().whileTrue(new SwerveDriveToPose(swerve, new Pose2d(8, 4, new Rotation2d(0.78539))));
        driver.getBottomButton().onTrue(new SwerveHome(swerve, new Pose2d(0, 4, new Rotation2d(0)) ));
        driver.getLeftButton().whileTrue(new SwerveDriveToPoseMP(swerve, new Pose2d(8, 4, new Rotation2d(0.78539))));
        driver.getTopButton().onTrue(new SwerveDriveHome(swerve));
    }

    /**************/
    /*** AUTONS ***/
    /**************/

    public void configureAutons() {
        autonChooser.setDefaultOption("Do Nothing", new DoNothingAuton());
        autonChooser.addOption("Mobility", new MobilityAuton(this));

        SmartDashboard.putData("Autonomous", autonChooser);
    }

    public Command getAutonomousCommand() {
        return autonChooser.getSelected();
    }
}
