package frc.robot.paths;

import frc.lib.drive.swerve.trajectory.SwerveTrajectoryGenerator;
import frc.robot.subsystems.Drivetrain;

public class ThreeBallForFivePath extends SwerveTrajectoryGenerator {

    public ThreeBallForFivePath(Drivetrain subsystem, double startRotation){
        // Setup
        super(subsystem.threeBallForFiveTrajectory);

        // Set the Start Rotation
        setStartRotation(startRotation);
        
        // addHeadingWaypoint(.2, 90);
        addHeadingWaypoint(.6, -130);
    }
}
