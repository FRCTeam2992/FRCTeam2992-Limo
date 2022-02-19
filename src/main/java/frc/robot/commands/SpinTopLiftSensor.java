// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TopLift;

public class SpinTopLiftSensor extends CommandBase {
  /** Creates a new SpinTopLift. */
  private TopLift mTopLift;

  private double mTopLiftSpeed;

  public SpinTopLiftSensor(TopLift subsystem, double topLiftSpeed) {
    mTopLift = subsystem;

    mTopLiftSpeed = topLiftSpeed;

    addRequirements(mTopLift);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (mTopLift.getSensorState()) {
      mTopLift.setTopLiftSpeed(0.0);
    } else {
      mTopLift.setTopLiftSpeed(mTopLiftSpeed);
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