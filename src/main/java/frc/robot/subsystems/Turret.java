// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turret extends SubsystemBase {

  //Turret Motors
  private WPI_TalonSRX turretMotor;

  /** Creates a new Turret. */
  public Turret() {
    turretMotor = new WPI_TalonSRX(14);
    turretMotor.setInverted(false);
    turretMotor.setNeutralMode(NeutralMode.Brake);
  }

  public void setTurretSpeed(double speed){
    turretMotor.set(ControlMode.PercentOutput, speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
