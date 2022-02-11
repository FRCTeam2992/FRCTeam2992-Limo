// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterHood extends SubsystemBase {

  private WPI_TalonSRX hoodMotor;

  private CANCoder hoodEncoder;


  public ShooterHood() {
    hoodMotor = new WPI_TalonSRX(20);
    hoodMotor.setInverted(true);
    hoodMotor.setNeutralMode(NeutralMode.Brake);

    hoodEncoder = new CANCoder(20);
    hoodEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
    hoodEncoder.configSensorDirection(true);

    hoodMotor.configRemoteFeedbackFilter(hoodEncoder, 20);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("encoder angle", getEncoderAngle());
    SmartDashboard.putNumber("hood angle", getHoodAngle());

  }

  public void setHoodSpeed(double speed) {
    hoodMotor.set(ControlMode.PercentOutput, speed);
  }

  public void setHoodPosition(double position){
  }

  public double getEncoderAngle() {
    double tempAngle = hoodEncoder.getAbsolutePosition() + Constants.hoodEncoderOffset;

    // if (tempAngle < -180.0){
    //   tempAngle += 360.0;
    // } else if (tempAngle > 180){
    //   tempAngle -= 360;
    // }
    
    if (tempAngle < 0.0){
        tempAngle += 360.0;
      } else if (tempAngle > 360.0){
        tempAngle -= 360;
      }
    return tempAngle;
  }

  public double getHoodAngle() {
    double angle = getEncoderAngle() * Constants.hoodAngleRatio;

    return angle;
  }

}
