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
    funnelMotor.setStatusFramePeriod(1, 255);
    funnelMotor.setStatusFramePeriod(2, 254);
    funnelMotor.setStatusFramePeriod(3, 253);
    funnelMotor.setStatusFramePeriod(4, 252);
    funnelMotor.setStatusFramePeriod(8, 251);
    funnelMotor.setStatusFramePeriod(10, 250);
    funnelMotor.setStatusFramePeriod(12, 249);
    funnelMotor.setStatusFramePeriod(13, 248);
    funnelMotor.setStatusFramePeriod(14, 247);
    funnelMotor.setStatusFramePeriod(21, 246);

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
