// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakeDeploy;

public class RetractIntake extends CommandBase {

  private IntakeDeploy mIntakeDeploy;

  private double mDeploySpeed;

  private boolean mPanicToggle;

  public RetractIntake(IntakeDeploy subsystem, double deploySpeed, boolean panicToggle) {

    mIntakeDeploy = subsystem;

    mDeploySpeed = deploySpeed;

    mPanicToggle = panicToggle;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (mPanicToggle){
      mIntakeDeploy.panicState = mIntakeDeploy.isDeployed;
    }

    mIntakeDeploy.deployIntake(mDeploySpeed);

    mIntakeDeploy.isDeployed = false;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
