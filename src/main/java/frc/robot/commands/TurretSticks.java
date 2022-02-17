// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
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
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double x;
    double y;

    x = -Robot.m_robotContainer.getController0().getLeftX();
    y = -Robot.m_robotContainer.getController0().getLeftY();

    double xyAngle = Math.atan2(y, x);

    double gyroValue = mTurret.navx.getYaw();

    double targetAngle = 360 - gyroValue + xyAngle;

    if (targetAngle > 360){
      targetAngle -= 360;
    }

    if (targetAngle < 0){
      targetAngle += 360;
    }
    SmartDashboard.putNumber("Joystick angle", xyAngle);
    SmartDashboard.putNumber("Joystick X", x);
    SmartDashboard.putNumber("Joystick Y", y);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
