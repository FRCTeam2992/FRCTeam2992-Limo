// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakeDeploy;

public class SpinIntake extends CommandBase {

  private Intake mIntake;
  private IntakeDeploy mIntakeDeploy;
  private double mIntakeSpeed;

  private boolean mIsPanic;

  public SpinIntake(Intake subsystem, IntakeDeploy subsystem2, double intakeSpeed, boolean isPanic) {
    mIntake = subsystem;
    mIntakeDeploy = subsystem2;
    mIntakeSpeed = intakeSpeed;

    mIsPanic = isPanic;
    addRequirements(mIntake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (mIsPanic){
      if (mIntakeDeploy.panicState){
        mIntake.setIntakeSpeed(mIntakeSpeed);
      }

      else {
        mIntake.setIntakeSpeed(0);
      }
    }
    mIntake.setIntakeSpeed(mIntakeSpeed);
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
