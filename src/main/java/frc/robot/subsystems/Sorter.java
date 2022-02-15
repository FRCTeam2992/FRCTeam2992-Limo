// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Sorter extends SubsystemBase {

  private WPI_VictorSPX bottomLiftMotor;
  private WPI_VictorSPX topLiftMotor;

  private DigitalInput topLiftSensor;
  private DigitalInput bottomLiftSensor;

  public Sorter() {
    bottomLiftMotor = new WPI_VictorSPX(42);
    bottomLiftMotor.setInverted(false);
    bottomLiftMotor.setNeutralMode(NeutralMode.Coast);

    topLiftMotor = new WPI_VictorSPX(42);
    topLiftMotor.setInverted(false);
    topLiftMotor.setNeutralMode(NeutralMode.Coast);

    addChild("topLiftMotor", topLiftMotor);

    addChild("bottomLiftMotor", bottomLiftMotor);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setTopLiftSpeed(double speed) {
    topLiftMotor.set(ControlMode.PercentOutput, speed);
  }

  public void setBottomLiftSpeed(double speed) {
    bottomLiftMotor.set(ControlMode.PercentOutput, speed);
  }
}
