package frc.robot.paths;

import frc.lib.drive.swerve.trajectory.SwerveTrajectoryGenerator;
import frc.robot.subsystems.Drivetrain;

public class FiveBallFinalPath extends SwerveTrajectoryGenerator {

    public FiveBallFinalPath(Drivetrain subsystem, double startRotation){
        // Setup
        super(subsystem.fiveBallFinalTrajectory);

        // Set the Start Rotation
        setStartRotation(startRotation);
    
        // addHeadingWaypoint(0.1, startRotation);
        addHeadingWaypoint(1.00, 135);
    }
}