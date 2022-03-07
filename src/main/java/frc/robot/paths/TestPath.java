package frc.robot.paths;

import frc.lib.drive.swerve.trajectory.SwerveTrajectoryGenerator;
import frc.robot.subsystems.Drivetrain;

public class TestPath extends SwerveTrajectoryGenerator {

    public TestPath(Drivetrain subsystem){
        // Setup
        super(subsystem.testPathTrajectory);

        // Set the Start Rotation
        setStartRotation(90.0);
    
        addHeadingWaypoint(0.1, 90.0);
        addTimedHeadingWaypoint(1.0, 2.0, 135.0);
    }
}
