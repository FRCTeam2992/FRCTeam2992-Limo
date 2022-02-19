/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;

public class StartShooter extends CommandBase {

  // cheating for a change
  // Subsystem Instance
  private Shooter mShooter;

  private double mMainShooterSpeed;
  private double mSecondaryShooterSpeed;

  public StartShooter(Shooter subsystem) {
    // Subsystem Instance
    mShooter = subsystem;

    // Set the Subsystem Requirement
    addRequirements(mShooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mMainShooterSpeed = mShooter.mainShooterSetSpeed;
    mSecondaryShooterSpeed = mShooter.secondaryShooterSetSpeed;
    mMainShooterSpeed = (mMainShooterSpeed / 600.0) * (Constants.shooterEncoderPulses * 0.75);
    mSecondaryShooterSpeed = (mSecondaryShooterSpeed / 600.0) * (Constants.shooterEncoderPulses);
    // SmartDashboard.putNumber("Commanded Main Speed", mMainShooterSpeed);
    // SmartDashboard.putNumber("Commanded Secondary Speed",
    // mSecondaryShooterSpeed);

    mShooter.setMainShooterVelocity(mMainShooterSpeed);
    mShooter.setSecondaryShooterVelocity(mSecondaryShooterSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mShooter.setMainShooterSpeed(0.0);
    mShooter.setSecondaryShooterSpeed(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
