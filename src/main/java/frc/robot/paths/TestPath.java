package frc.robot.paths;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.lib.drive.swerve.trajectory.SwerveTrajectoryGenerator;
import frc.robot.subsystems.Drivetrain;

public class TestPath extends SwerveTrajectoryGenerator {

    public TestPath(Drivetrain subsystem){
        // Setup
        super(subsystem.testPathTrajectory);

        // Set the Start Rotation
        setStartRotation(180.0);
    
        addHeadingWaypoint(0.1, 180.0);
        addTimedHeadingWaypoint(1.0, 2.0, 225.0);
    }
}
