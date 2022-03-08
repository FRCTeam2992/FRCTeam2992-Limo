// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BottomLift extends SubsystemBase {

  private WPI_VictorSPX bottomLiftMotor;

  private DigitalInput liftSensor1;
  private DigitalInput liftSensor2;

  public BottomLift() {
    bottomLiftMotor = new WPI_VictorSPX(23);
    bottomLiftMotor.setInverted(false);
    bottomLiftMotor.setNeutralMode(NeutralMode.Brake);
    bottomLiftMotor.setStatusFramePeriod(1, 255);
    bottomLiftMotor.setStatusFramePeriod(2, 254);
    bottomLiftMotor.setStatusFramePeriod(3, 253);
    bottomLiftMotor.setStatusFramePeriod(4, 252);
    bottomLiftMotor.setStatusFramePeriod(8, 251);
    bottomLiftMotor.setStatusFramePeriod(10, 250);
    bottomLiftMotor.setStatusFramePeriod(12, 249);
    bottomLiftMotor.setStatusFramePeriod(13, 248);
    bottomLiftMotor.setStatusFramePeriod(14, 247);
    bottomLiftMotor.setStatusFramePeriod(21, 246);

    addChild("bottomLiftMotor", bottomLiftMotor);

    liftSensor1 = new DigitalInput(0);
    liftSensor2 = new DigitalInput(1);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.putBoolean("sensor 1", getSensor1State());
    // SmartDashboard.putBoolean("sensor 2", getSensor2State());


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
