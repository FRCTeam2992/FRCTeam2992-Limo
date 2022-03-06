// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterHood extends SubsystemBase {

  private WPI_TalonSRX hoodMotor;

  private CANCoder hoodEncoder;

  public double hoodPosition = Constants.defaultHoodPosition;

  public PIDController hoodPID;

  public MedianFilter hoodMedianFilter;

  public ShooterHood() {
    hoodMotor = new WPI_TalonSRX(33);
    hoodMotor.setInverted(true);
    hoodMotor.setNeutralMode(NeutralMode.Brake);

    addChild("hoodMotor", hoodMotor);

    hoodEncoder = new CANCoder(33, "CanBus2");

    hoodEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
    hoodEncoder.configSensorDirection(true);

    hoodPID = new PIDController(Constants.hoodP, Constants.hoodI, Constants.hoodD);
    hoodPID.setTolerance(Constants.hoodTolerance);
    hoodPID.disableContinuousInput();
    hoodPID.setIntegratorRange(-0.2, 0.2);

    hoodMotor.configRemoteFeedbackFilter(hoodEncoder, 0);

    hoodMedianFilter = new MedianFilter(5);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("Hood Encoder Angle", getEncoderAngle());
  }

  public void setHoodSpeed(double speed) {

    hoodMotor.set(ControlMode.PercentOutput, speed);
  }

  public void setHoodPosition(double position) {
    position = Math.max(position, Constants.minHoodPosition);
    position = Math.min(position, Constants.maxHoodPosition);

    if(Math.abs(getEncoderAngle() - position) > 15.0){
      hoodPID.reset();
    }

    double power = hoodPID.calculate(getEncoderAngle(), position) + Constants.hoodF;
    power = Math.min(power, .5);
    power = Math.max(power, -.4);

    hoodMotor.set(ControlMode.PercentOutput, power);
  }

  public double getEncoderAngle() {
    double tempAngle = hoodEncoder.getAbsolutePosition() + Constants.hoodEncoderOffset;

    if (tempAngle < -180.0) {
      tempAngle += 360.0;
    } else if (tempAngle > 180) {
      tempAngle -= 360;
    }

    

    // if (tempAngle < 0.0){
    // tempAngle += 360.0;
    // } else if (tempAngle > 360.0){
    // tempAngle -= 360;
    // }
    return hoodMedianFilter.calculate(tempAngle);
  }

  public double getHoodAngle() {
    double angle = getEncoderAngle() * Constants.hoodAngleRatio;

    return angle;
  }
}
