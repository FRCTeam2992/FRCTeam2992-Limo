// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BottomLift extends SubsystemBase {

  private WPI_VictorSPX bottomLiftMotor;

  private DigitalInput liftSensor1;
  private DigitalInput liftSensor2;

  public BottomLift() {
    bottomLiftMotor = new WPI_VictorSPX(23);
    bottomLiftMotor.setInverted(false);
    bottomLiftMotor.setNeutralMode(NeutralMode.Brake);

    addChild("bottomLiftMotor", bottomLiftMotor);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setBottomLiftSpeed(double speed) {
    bottomLiftMotor.set(ControlMode.PercentOutput, speed);
  }

  public boolean getSensor1State() {
    return liftSensor1.get();
  }

  public boolean getSensor2State() {
    return liftSensor2.get();
  }
}
