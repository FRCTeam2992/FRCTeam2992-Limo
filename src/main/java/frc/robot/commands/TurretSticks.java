// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.Turret;

public class TurretSticks extends CommandBase {

  private Turret mTurret;

  /** Creates a new TurretSticks. */
  public TurretSticks(Turret subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    mTurret = subsystem;

    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double x = -Robot.mRobotContainer.controller1.getLeftX();
    double y = -Robot.mRobotContainer.controller1.getLeftY();
    double targetAngle;
    double xyMagnitude = Math.sqrt((x * x) + (y * y));

    if (xyMagnitude >= Constants.turretJoystickDeadband) {
      if(xyMagnitude > 1){
        x /= xyMagnitude;
        y /= xyMagnitude;
      }
      if(Constants.isFieldCentric){
      targetAngle = Turret.angleOverlap((Math.toDegrees(Math.atan2(y, x)) - 90) - mTurret.getGyroYaw());
      }
      mTurret.goToAngle(Turret.angleOverlap(targetAngle));
      // SmartDashboard.putNumber("TurretStick output", targetAngle);
    } else {
      mTurret.stopTurret();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mTurret.stopTurret();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
