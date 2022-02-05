// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands; 

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.drive.swerve.trajectory.SwerveTrajectory;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class AutoFollowPath extends CommandBase {

  // Subsystem Instance
  private Drivetrain mDriveTrain;

  // Trajectory Instance
  private SwerveTrajectory mSwerveyTrajectory;
  private Trajectory mTrajectory;

  // Drive Controller Instance
  private HolonomicDriveController controller;

  // Timer
  private Timer elapsedTimer;

  public AutoFollowPath(Drivetrain subsystem, SwerveTrajectory swerveTrajectory) {
    // Subsystem Instance
    mDriveTrain = subsystem;

    // Set the Subsystem Requirement
    addRequirements(mDriveTrain);

    // Save the Swerve Trajectory
    mSwerveyTrajectory = swerveTrajectory;

    // Get the Trajectory
    mTrajectory = mSwerveyTrajectory.getTrajectory();

    // Create the Theta Controller
    ProfiledPIDController thetaController = new ProfiledPIDController(Constants.thetaCorrectionP,
        Constants.thetaCorrectionI, Constants.thetaCorrectionD,
        new TrapezoidProfile.Constraints(Constants.maxThetaVelocity, Constants.maxThetaAcceleration));

    // Enable Wrapping on the Theta Controller (Radians)
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    // Initialize the Drive Controller
    controller = new HolonomicDriveController(
        new PIDController(Constants.xCorrectionP, Constants.xCorrectionI, Constants.xCorrectionD),
        new PIDController(Constants.yCorrectionP, Constants.yCorrectionI, Constants.yCorrectionD), thetaController);

    // Timer
    elapsedTimer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Get the Trajectory Start Pose
    Pose2d trajectoryStartPose = mTrajectory.getInitialPose();

    // Set the Odometry Position to the Trajectory Start Position
    mDriveTrain.setOdometryPosition(new Pose2d(trajectoryStartPose.getX(), trajectoryStartPose.getY(),
        Rotation2d.fromDegrees(-mDriveTrain.navx.getYaw())));

    // Reset and Start the Elapsed Timer
    elapsedTimer.reset();
    elapsedTimer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Get the Current Time
    double currentTime = elapsedTimer.get();

    // Get the Latest State
    Trajectory.State latestState = mTrajectory.sample(currentTime);

    // Get the Desired Heading
    double heading = mSwerveyTrajectory.getDesiredHeading(currentTime);

    // Get the Ajusted Speeds
    ChassisSpeeds adjustSpeeds = controller.calculate(mDriveTrain.latestSwervePose, latestState,
        Rotation2d.fromDegrees(heading));

    // Get the Module States
    SwerveModuleState[] moduleStates = mDriveTrain.swerveDriveKinematics.toSwerveModuleStates(adjustSpeeds);

    // Set the Module States
    mDriveTrain.frontLeftModule.setState(moduleStates[0]);
    mDriveTrain.frontRightModule.setState(moduleStates[1]);
    mDriveTrain.rearLeftModule.setState(moduleStates[2]);
    mDriveTrain.rearRightModule.setState(moduleStates[3]);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mDriveTrain.stopDrive();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return elapsedTimer.get() >= mTrajectory.getTotalTimeSeconds();
  }
}