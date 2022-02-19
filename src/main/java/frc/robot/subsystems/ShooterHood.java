// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterHood extends SubsystemBase {

  private WPI_TalonSRX hoodMotor;

  private CANCoder hoodEncoder;

  public PIDController hoodPID;

  public ShooterHood() {
    hoodMotor = new WPI_TalonSRX(33);
    hoodMotor.setInverted(true);
    hoodMotor.setNeutralMode(NeutralMode.Brake);

    hoodEncoder = new CANCoder(33);
    hoodEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
    hoodEncoder.configSensorDirection(true);

    hoodPID = new PIDController(Constants.hoodP, Constants.hoodI, Constants.hoodD);
    hoodPID.setTolerance(Constants.hoodTolerance);
    hoodPID.disableContinuousInput();

    hoodMotor.configRemoteFeedbackFilter(hoodEncoder, 33);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("encoder angle", getEncoderAngle());
  }

  public void setHoodSpeed(double speed) {

    hoodMotor.set(ControlMode.PercentOutput, speed);
  }

  public void setHoodPosition(double position) {
    position = Math.min(position, Constants.minHoodPosition);
    position = Math.max(position, Constants.maxHoodPosition);
    setHoodSpeed(hoodPID.calculate(getEncoderAngle(), position));
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
    return tempAngle;
  }

  public double getHoodAngle() {
    double angle = getEncoderAngle() * Constants.hoodAngleRatio;

    return angle;
  }
}
