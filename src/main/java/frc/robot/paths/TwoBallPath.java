package frc.robot.paths;

import frc.lib.drive.swerve.trajectory.SwerveTrajectoryGenerator;
import frc.robot.subsystems.Drivetrain;

public class TwoBallPath extends SwerveTrajectoryGenerator {

    public TwoBallPath(Drivetrain subsystem, double startRotation){
        // Setup
        super(subsystem.fiveBallTrajectory);

        // Set the Start Rotation
        setStartRotation(startRotation);
    
        //addHeadingWaypoint(0.1, startRotation);
        addHeadingWaypoint(0.1, 230.0);
        addHeadingWaypoint(1, 230);
    }
}
