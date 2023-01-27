package com.stuypulse.robot.commands.auton;

import com.pathplanner.lib.PathPlanner;
import com.stuypulse.robot.RobotContainer;
import com.stuypulse.robot.commands.swerve.BasicGyroEngage;
import com.stuypulse.robot.commands.swerve.FollowTrajectory;
import com.stuypulse.robot.constants.Settings.Swerve.Motion;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class TwoPieceDock extends SequentialCommandGroup {

    public TwoPieceDock(RobotContainer robot) {
        addCommands(
            new FollowTrajectory(
                robot.swerve,
                PathPlanner.loadPath("2 Piece + Dock", Motion.CONSTRAINTS)
            ).robotRelative(),
            new BasicGyroEngage(robot.swerve)
        );
    }

}
