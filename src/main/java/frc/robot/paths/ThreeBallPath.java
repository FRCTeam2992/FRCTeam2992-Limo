package frc.robot.paths;

import frc.lib.drive.swerve.trajectory.SwerveTrajectoryGenerator;
import frc.robot.subsystems.Drivetrain;

public class ThreeBallPath extends SwerveTrajectoryGenerator {

    public ThreeBallPath(Drivetrain subsystem, double startRotation){
        // Setup
        super(subsystem.threeBallTrajectory);

        // Set the Start Rotation
        setStartRotation(startRotation);
    
        addHeadingWaypoint(0.1, startRotation);
    }
}
