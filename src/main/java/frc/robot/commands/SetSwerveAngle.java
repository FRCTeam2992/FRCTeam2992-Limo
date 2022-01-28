// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.drive.swerve.SwerveModuleFalconFalcon;
import frc.robot.subsystems.Drivetrain;

public class SetSwerveAngle extends CommandBase {
  private Drivetrain mDrivetrain;

  private double mAngle;

  public SetSwerveAngle(Drivetrain subsystem, double angle) {
    mDrivetrain = subsystem;

    mAngle = angle;
    addRequirements(mDrivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mDrivetrain.frontLeftModule.setTurnAngle(mAngle);
    mDrivetrain.frontRightModule.setTurnAngle(mAngle);
    mDrivetrain.rearLeftModule.setTurnAngle(mAngle);
    mDrivetrain.rearRightModule.setTurnAngle(mAngle);
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
