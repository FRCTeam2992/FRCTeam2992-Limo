// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterHood;

public class HoldHoodAngle extends CommandBase {
  /** Creates a new HoldHoodAngle. */
  private ShooterHood mShooterHood;

  private double startHoodAngle;

  public HoldHoodAngle(ShooterHood subsystem) {
    mShooterHood = subsystem;

    addRequirements(mShooterHood);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startHoodAngle = mShooterHood.getEncoderAngle();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mShooterHood.setHoodPosition(startHoodAngle);
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
