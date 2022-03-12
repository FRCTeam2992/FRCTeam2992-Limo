package frc.robot.paths;

import frc.lib.drive.swerve.trajectory.SwerveTrajectoryGenerator;
import frc.robot.subsystems.Drivetrain;

public class FiveBallPath extends SwerveTrajectoryGenerator {

    public FiveBallPath(Drivetrain subsystem, double startRotation){
        // Setup
        super(subsystem.fiveBallTrajectory);

        // Set the Start Rotation
        setStartRotation(startRotation);
    
        addHeadingWaypoint(0.1, startRotation);
    }
}