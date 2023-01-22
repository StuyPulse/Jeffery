package com.stuypulse.robot.commands.alignment;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.stuypulse.robot.commands.swerve.FollowTrajectory;
import com.stuypulse.robot.constants.Settings.Swerve.*;
import com.stuypulse.robot.subsystems.SwerveDrive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class FollowTrajectoryAlign extends SequentialCommandGroup {
   
    private PathPlannerTrajectory path;

    public FollowTrajectoryAlign(SwerveDrive swerve, Pose2d targetPose){
        path = PathPlanner.generatePath(
            new PathConstraints(Chassis.MAX_SPEED, Chassis.MAX_ACCELERATION),
            new PathPoint(swerve.getTranslation(), swerve.getAngle(), swerve.getAngle()),
            new PathPoint(targetPose.getTranslation(), targetPose.getRotation(), targetPose.getRotation())
        );
        //heading: direction of movement, as if you have another point after (direction leaving the point)
        //holonomic rotation: where it's facing 

        addCommands(new FollowTrajectory(swerve, path));
        addRequirements(swerve);  
    }
}
