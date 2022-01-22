// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class ChangeIntakeSpeed extends CommandBase {

  private Intake mIntake;

  private double mChangeSpeed;

  /** Creates a new ChangeIntakeSpeed. */
  public ChangeIntakeSpeed(Intake subsystem, double changeSpeed) {

    mIntake = subsystem;

    mChangeSpeed = changeSpeed; 

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double changeSpeed = mIntake.intakeSetSpeed + mChangeSpeed;
    
    changeSpeed = Math.max(0, changeSpeed);

    mIntake.setIntakeMotor(changeSpeed);
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
