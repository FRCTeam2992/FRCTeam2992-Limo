// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterHood;

public class MoveHood extends CommandBase {
  private ShooterHood mShooterHood;

  private double mHoodSpeed;

  public MoveHood(ShooterHood subsystem, double hoodSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    mShooterHood = subsystem;

    mHoodSpeed = hoodSpeed;

    addRequirements(mShooterHood);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if ((mHoodSpeed < 0.0 && ((
      mShooterHood.getEncoderAngle() < Constants.minHoodPosition) && (
      mShooterHood.getEncoderAngle() > Constants.minHoodPositionOverlap))) || (mHoodSpeed > 0.0 && ((
          mShooterHood.getEncoderAngle() > Constants.maxHoodPosition) && (
          mShooterHood.getEncoderAngle() < Constants.maxHoodPositionOverlap)
          ))) {
      mShooterHood.setHoodSpeed(0.0);

    } else {
      mShooterHood.setHoodSpeed(mHoodSpeed);

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