/************************ PROJECT PHIL ************************/
/* Copyright (c) 2023 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.stuypulse.robot.commands.auton.DoNothingAuton;
import com.stuypulse.robot.commands.auton.LongAuton;
import com.stuypulse.robot.commands.auton.MobilityAuton;
import com.stuypulse.robot.commands.auton.OnePiece;
import com.stuypulse.robot.commands.auton.OnePieceDock;
import com.stuypulse.robot.commands.auton.ThreePiece;
import com.stuypulse.robot.commands.auton.ThreePieceDock;
import com.stuypulse.robot.commands.auton.TwoPieceDock;
import com.stuypulse.robot.commands.drivetrain.DrivetrainDriveCommand;
import com.stuypulse.robot.commands.drivetrain.DrivetrainRamseteCommand;
import com.stuypulse.robot.commands.drivetrain.DrivetrainAlignCommand;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Alignment;
import com.stuypulse.robot.subsystems.Drivetrain;
import com.stuypulse.robot.subsystems.ICamera;
import com.stuypulse.robot.subsystems.camera.LLCamera;
import com.stuypulse.robot.subsystems.camera.PVCamera;
import com.stuypulse.robot.commands.SwerveDriveToPose;
import com.stuypulse.robot.commands.SwerveDriveToPoseMP;
import com.stuypulse.robot.commands.auton.DoNothingAuton;
import com.stuypulse.robot.commands.swerve.BasicGyroEngage;
import com.stuypulse.robot.commands.swerve.ChargeDrive;
import com.stuypulse.robot.commands.swerve.GyroAutoEngage;
import com.stuypulse.robot.commands.swerve.LockWheels;
import com.stuypulse.robot.commands.swerve.SamAutoEngage;
import com.stuypulse.robot.commands.swerve.SetpointAutoEngage;
import com.stuypulse.robot.commands.swerve.SwerveDriveDrive;
import com.stuypulse.robot.commands.swerve.SwerveDriveHome;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.subsystems.SwerveDrive;
import com.stuypulse.robot.util.BootlegXbox;
import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.input.gamepads.AutoGamepad;
import com.stuypulse.stuylib.input.gamepads.Xbox;
import com.stuypulse.stuylib.network.SmartNumber;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class RobotContainer {

    // Gamepads
    public final Gamepad driver = new BootlegXbox(Ports.Gamepad.DRIVER);
    public final Gamepad operator = new BootlegXbox(Ports.Gamepad.OPERATOR);

    // Subsystem
    public final ICamera camera = ICamera.getInstance();
    public final SwerveDrive swerve = SwerveDrive.getInstance(camera);

    // public final Drivetrain drivetrain = new Drivetrain(null);

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
        // drivetrain.setDefaultCommand(new DrivetrainDriveCommand(drivetrain, driver));
        swerve.setDefaultCommand(new SwerveDriveDrive(swerve, driver));
    }

    /***************/
    /*** BUTTONS ***/
    /***************/

    private void configureButtonBindings() {

        driver.getRightBumper()
            .whileTrue(new BasicGyroEngage(swerve));

        // Reset Position
        driver.getTopButton()
                .onTrue(new InstantCommand(
                        // () -> drivetrain.setPose(new Pose2d(5.68, -3.36, new Rotation2d(Math.toRadians(10)))),
                        // () -> swerve.reset(Settings.STARTING_POSE))
                        () -> swerve.reset(new Pose2d(new SmartNumber("Auto Engage/setpoint", 2.616).get(), 0, new Rotation2d())))
                        );
        
        // Auto Engage
        driver.getDPadUp().whileTrue(new BasicGyroEngage(swerve));
        driver.getDPadLeft().whileTrue(new GyroAutoEngage(swerve));
        driver.getDPadDown().whileTrue(new SetpointAutoEngage(swerve));
        driver.getDPadRight().whileTrue(new SamAutoEngage(swerve));
        
        driver.getBottomButton()
            .whileTrue(new ChargeDrive(swerve, driver));
        
        driver.getLeftBumper()
            .whileTrue(new LockWheels(swerve));
              

        // driver.getBottomButton().onTrue(new DrivetrainAlignCommand(drivetrain, camera, Alignment.TARGET_POSE));

        // driver.getTopButton().onTrue(new Command(()))
        // new Pose2d(5.68, -3.36, new Rotation2d(Math.toRadians(10)))

        driver.getRightButton().whileTrue(new SwerveDriveToPose(swerve, new Pose2d(0, 0, new Rotation2d(0))));
        // driver.getBottomButton().onTrue(new SwerveDriveHome(swerve, new Pose2d(0, 4, new Rotation2d(0)) ));
        driver.getLeftButton().whileTrue(new SwerveDriveToPoseMP(swerve, new Pose2d(0, 0, new Rotation2d(0))));
        driver.getTopButton().onTrue(new SwerveDriveHome(swerve));

    }

    /**************/
    /*** AUTONS ***/
    /**************/

    public void configureAutons() {
        autonChooser.setDefaultOption("Do Nothing", new DoNothingAuton());
        autonChooser.addOption("Mobility", new MobilityAuton(this));
        autonChooser.addOption("1 Piece + Dock", new OnePieceDock(this));
        autonChooser.addOption("2 Piece + Dock", new TwoPieceDock(this));
        autonChooser.addOption("3 Piece + Dock", new ThreePieceDock(this));
        autonChooser.addOption("1 Piece", new OnePiece(this));
        autonChooser.addOption("3 Piece", new ThreePiece(this));
        autonChooser.addOption("Long Auton", new LongAuton(this));

        SmartDashboard.putData("Autonomous", autonChooser);
    }

    public Command getAutonomousCommand() {
        return autonChooser.getSelected();
    }
}
