// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.Climb;

public class ClimbSticks extends CommandBase {
  //Subsystem
  private Climb mClimb;

  private double climbY;


  public ClimbSticks(Climb subsystem) {

    mClimb = subsystem;
    
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   
    climbY = -Robot.m_robotContainer.controller1.getLeftY();

    if (Math.abs(climbY) <= Constants.joystickDeadband*2){
      climbY = 0.0;
    }      
    climbY = climbY * climbY * climbY;
    mClimb.moveClimb(climbY);
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

