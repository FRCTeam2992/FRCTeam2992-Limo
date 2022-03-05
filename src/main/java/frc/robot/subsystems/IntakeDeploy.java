// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeDeploy extends SubsystemBase {

  private CANSparkMax intakeDeployMotor;

  public boolean isDeployed = false;

  public boolean panicState;
  
  /** Creates a new IntakeDeploy. */
  public IntakeDeploy() {

    intakeDeployMotor = new CANSparkMax(25, MotorType.kBrushless);
    intakeDeployMotor.setIdleMode(IdleMode.kBrake);
  }
  
  public void deployIntake(double deploySpeed) {
    intakeDeployMotor.set(deploySpeed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
