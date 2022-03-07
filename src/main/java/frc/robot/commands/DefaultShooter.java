// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Shooter;

public class DefaultShooter extends CommandBase {
  /** Creates a new StopIntake. */
  private Shooter mShooter;
  
  public DefaultShooter(Shooter shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    mShooter = shooter;
    addRequirements(mShooter);
  }

  // Called when the command is initially scheduled.
  @Override

  public void initialize() {
   
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (mShooter.isShooterCommanded()) {
      // Shooterwas last commanded on so spin it
      CommandScheduler.getInstance().schedule(new StartShooter(mShooter));
    } else {
      CommandScheduler.getInstance().schedule(new StopShooter(mShooter));
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
