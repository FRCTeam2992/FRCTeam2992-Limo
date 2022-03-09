package frc.robot.paths;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.drive.swerve.trajectory.SwerveTrajectoryGenerator;
import frc.robot.Constants;

public class StraightPath extends SwerveTrajectoryGenerator {

        public StraightPath(double distance) {
                // Setup
                super(new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0)),
                                new Pose2d(distance, 0.0, Rotation2d.fromDegrees(0.0)), Constants.maxPathFollowingVelocity,
                                Constants.maxPathFollowingAcceleration);

                // Heading Waypoints
                addHeadingWaypoint(0.0, 0.0);
        }
}
