// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BottomLift extends SubsystemBase {

  private WPI_VictorSPX bottomLiftMotor;

  private boolean isCommanded = false;        // Is commanded to be moving during default command
  private boolean checkSensor = false;        // CHeck the ball sensor if commanded
  private double  noBallSpeed = 0.0;        // Speed to be commanded if no ball is seen
  private double  withBallSpeed = 0.0;      // Speed to be commanded if we see a ball
  private double  sensorDelay = 0.0;        // How long after seeing a ball before we jump to the withBallSpeed

  private Timer sensorTimer;

  private DigitalInput liftSensor1;
  private DigitalInput liftSensor2;

  public BottomLift() {
    bottomLiftMotor = new WPI_VictorSPX(23);
    bottomLiftMotor.setInverted(false);
    bottomLiftMotor.setNeutralMode(NeutralMode.Brake);
    bottomLiftMotor.setStatusFramePeriod(1, 255);
    bottomLiftMotor.setStatusFramePeriod(2, 255);
    bottomLiftMotor.setStatusFramePeriod(3, 255);
    bottomLiftMotor.setStatusFramePeriod(4, 255);
    bottomLiftMotor.setStatusFramePeriod(8, 255);
    bottomLiftMotor.setStatusFramePeriod(10, 255);
    bottomLiftMotor.setStatusFramePeriod(12, 255);
    bottomLiftMotor.setStatusFramePeriod(13, 255);
    bottomLiftMotor.setStatusFramePeriod(14, 255);
    bottomLiftMotor.setStatusFramePeriod(21, 255);

    addChild("bottomLiftMotor", bottomLiftMotor);

    liftSensor1 = new DigitalInput(0);
    liftSensor2 = new DigitalInput(1);

    sensorTimer = new Timer();
    sensorTimer.reset();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.putBoolean("sensor 1", getSensor1State());
    // SmartDashboard.putBoolean("sensor 2", getSensor2State());
    if (!getSensor1State() && !getSensor2State()) {
      // Neither sensor sees a ball -- cargo lift is empty to reset timer
      sensorTimer.reset();
    }

  }

  public void setBottomLiftSpeed(double speed) {
    bottomLiftMotor.set(ControlMode.PercentOutput, speed);
  }

  public void setBottomListSpeedAsCommandedSensor() {
    if (isCommanded()) {
      // We should be running in default command mode
      if ((!getSensor1State() && !getSensor2State()) || (sensorTimer.get() < sensorDelay)) {
        // No ball is seen so run at higher speed
        bottomLiftMotor.set(ControlMode.PercentOutput, noBallSpeed);
      } else {
        // We have a ball in cargo lift for long enough so change speed
        bottomLiftMotor.set(ControlMode.PercentOutput, withBallSpeed);
      }
    } else {
      // We are commanded off
      bottomLiftMotor.set(ControlMode.PercentOutput, 0.0);
    }
  }

  public boolean getSensor1State() {
    return liftSensor1.get();
  }

  public boolean getSensor2State() {
    return liftSensor2.get();
  }

  public boolean isCommanded() {
    return isCommanded;
  }

  public void setCommanded(boolean isCommanded) {
    this.isCommanded = isCommanded;
  }

  public boolean isCheckSensor() {
    return checkSensor;
  }

  public void setCheckSensor(boolean checkSensor) {
    this.checkSensor = checkSensor;
  }

  public double getNoBallSpeed() {
    return noBallSpeed;
  }

  public void setNoBallSpeed(double noBallSpeed) {
    this.noBallSpeed = noBallSpeed;
  }

  public double getWithBallSpeed() {
    return withBallSpeed;
  }

  public void setWithBallSpeed(double withBallSpeed) {
    this.withBallSpeed = withBallSpeed;
  }

  public double getSensorDelay() {
    return sensorDelay;
  }

  public void setSensorDelay(double sensorDelay) {
    this.sensorDelay = sensorDelay;
  }

  
}
