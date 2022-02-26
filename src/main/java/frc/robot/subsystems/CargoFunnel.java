// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CargoFunnel extends SubsystemBase {

  private WPI_VictorSPX funnelMotor;

  public CargoFunnel() {

    funnelMotor = new WPI_VictorSPX(22);
    funnelMotor.setInverted(true);
    funnelMotor.setNeutralMode(NeutralMode.Coast);

    addChild("funnelMotor", funnelMotor);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setFunnelSpeed(double speed) {
    funnelMotor.set(ControlMode.PercentOutput, speed);
  }
}
