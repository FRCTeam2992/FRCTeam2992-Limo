// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BottomLift;
import frc.robot.subsystems.CargoFunnel;
import frc.robot.subsystems.IntakeDeploy;

public class SpinCargoFunnelSensor extends CommandBase {
  /** Creates a new SpinCargoFunnel. */
  private CargoFunnel mCargoFunnel;
  private BottomLift mBottomLift;

  private double mCargoFunnelSpeed;
  private double mCargoFunnelSensorSpeed;

  private IntakeDeploy mIntakeDeploy;

  private boolean mIsPanic;


  public SpinCargoFunnelSensor(CargoFunnel CFSubsystem, BottomLift BLSubsystem, IntakeDeploy IDSubsystem, double cargoFunnelSpeed, double cargoFunnelSensorSpeed, boolean isPanic) {
    mCargoFunnel = CFSubsystem;
    mBottomLift = BLSubsystem;
    mCargoFunnelSpeed = cargoFunnelSpeed;
    mCargoFunnelSensorSpeed = cargoFunnelSensorSpeed;
    mIntakeDeploy = IDSubsystem;
    mIsPanic = isPanic;

    addRequirements(mCargoFunnel);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (mIsPanic){
      if(mIntakeDeploy.panicState){
        if (mBottomLift.getSensor1State() || mBottomLift.getSensor2State()) {
          mCargoFunnel.setFunnelSpeed(mCargoFunnelSensorSpeed);
    
        } else {
          mCargoFunnel.setFunnelSpeed(mCargoFunnelSpeed);
        }
      }

      else {
        mCargoFunnel.setFunnelSpeed(0.0);
      }
    }

    if (mBottomLift.getSensor1State() || mBottomLift.getSensor2State()) {
      mCargoFunnel.setFunnelSpeed(mCargoFunnelSensorSpeed);

    } else {
      mCargoFunnel.setFunnelSpeed(mCargoFunnelSpeed);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mCargoFunnel.setFunnelSpeed(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
