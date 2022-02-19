// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Turret;

public class TurretSticks extends CommandBase {

  private Turret mTurret;
  private Climb mClimb;

  /** Creates a new TurretSticks. */
  public TurretSticks(Turret subsystem, Climb climbSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    mTurret = subsystem;
    mClimb = climbSubsystem;

    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double gyroValue = mTurret.navx.getYaw();

    double x = -Robot.m_robotContainer.controller0.getLeftX();
    double y = -Robot.m_robotContainer.controller0.getLeftY();
    double targetAngle;

    if ((Math.abs(x) > Constants.turretJoystickDeadband
        || Math.abs(y) > Constants.turretJoystickDeadband) && mClimb.toggleClimbMode) {
      targetAngle = 360 - gyroValue + (mTurret.angleOverlap(Math.toDegrees(Math.atan2(y, x)) - 90));

      mTurret.goToAngle(mTurret.angleOverlap(targetAngle));
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
