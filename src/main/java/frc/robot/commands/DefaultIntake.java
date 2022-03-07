// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Intake;

public class DefaultIntake extends CommandBase {
  /** Creates a new StopIntake. */
  private Intake mIntake;
  
  public DefaultIntake(Intake intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    mIntake = intake;
    addRequirements(mIntake);
  }

  // Called when the command is initially scheduled.
  @Override

  public void initialize() {
   
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (mIntake.getIntakeCommanded()) {
      // Intake was last commanded on so spin the Intake
      CommandScheduler.getInstance().schedule(new SpinIntake(mIntake, 0.5));
    } else {
      CommandScheduler.getInstance().schedule(new StopIntake(mIntake));
    }
  }

  // Called once the command ends or is interrupted
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
