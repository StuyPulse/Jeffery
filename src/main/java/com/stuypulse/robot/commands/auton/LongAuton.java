package com.stuypulse.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.stuypulse.robot.RobotContainer;
import com.stuypulse.robot.commands.swerve.FollowTrajectory;

public class LongAuton extends SequentialCommandGroup {
    public LongAuton(RobotContainer robot) {
        var eventmap = new HashMap<String, Command>();
        eventmap.put("long", new SequentialCommandGroup(new PrintCommand("hello world"), new WaitCommand(2), new PrintCommand("FART CIOCK SIT FUCK ASS")));
        eventmap.put("bob", new SequentialCommandGroup(new PrintCommand("bob says hello world"), new WaitCommand(2), new PrintCommand("bob enjoys FART CIOCK SIT FUCK ASS")));
        List<PathPlannerTrajectory> group = PathPlanner.loadPathGroup("Mobility", new PathConstraints(3,3));
        addCommands(
            // new FollowTrajectory(
            //     robot.swerve,
            //     PathPlanner.loadPath("Mobility", Motion.CONSTRAINTS)
            // ).robotRelative().withEvents(eventmap),
            new FollowTrajectory(robot.swerve, group.get(0)).robotRelative().withEvents(eventmap),
            new WaitCommand(1),
            new PrintCommand("bangin"),
            new FollowTrajectory(robot.swerve, group.get(1)).robotRelative().withEvents(eventmap)
        );
    }
}
