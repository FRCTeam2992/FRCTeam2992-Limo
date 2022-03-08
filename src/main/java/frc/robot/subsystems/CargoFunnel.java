// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CargoFunnel extends SubsystemBase {

  private WPI_VictorSPX funnelMotor;

  private boolean isCommanded = false;        // Is commanded to be moving during default command
  private boolean checkSensor = false;        // CHeck the ball sensor if commanded
  private double  noBallSpeed = 0.0;        // Speed to be commanded if no ball is seen
  private double  withBallSpeed = 0.0;      // Speed to be commanded if we see a ball
  private double  sensorDelay = 0.0;        // How long after seeing a ball before we jump to the withBallSpeed

  private Timer sensorTimer;          // Timer to keep track of how long we have seen a ball in cargo lift
  private BottomLift mBottomLift;

  public CargoFunnel(BottomLift bottomLift) {

    funnelMotor = new WPI_VictorSPX(22);
    funnelMotor.setInverted(true);
    funnelMotor.setNeutralMode(NeutralMode.Coast);
    funnelMotor.setStatusFramePeriod(1, 255);
    funnelMotor.setStatusFramePeriod(2, 255);
    funnelMotor.setStatusFramePeriod(3, 255);
    funnelMotor.setStatusFramePeriod(4, 255);
    funnelMotor.setStatusFramePeriod(8, 255);
    funnelMotor.setStatusFramePeriod(10, 255);
    funnelMotor.setStatusFramePeriod(12, 255);
    funnelMotor.setStatusFramePeriod(13, 255);
    funnelMotor.setStatusFramePeriod(14, 255);
    funnelMotor.setStatusFramePeriod(21, 255);

    addChild("funnelMotor", funnelMotor);

    sensorTimer = new Timer();
    mBottomLift = bottomLift;
    sensorTimer.reset();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (!mBottomLift.getSensor1State() && !mBottomLift.getSensor2State()) {
      // Neither sensor sees a ball -- cargo lift is empty to reset timer
      sensorTimer.reset();
    }

  }

  public void setFunnelSpeed(double speed) {
    funnelMotor.set(ControlMode.PercentOutput, speed);
  }

  public void setFunnelSpeedUsingCommandedSensors() {
    if (isCommanded()) {
      // We should be running in default command mode
      if ((!mBottomLift.getSensor1State() && !mBottomLift.getSensor2State()) || (sensorTimer.get() < sensorDelay)) {
        // No ball is seen so run at higher speed
        funnelMotor.set(ControlMode.PercentOutput, noBallSpeed);
      } else {
        // We have a ball in cargo lift for long enough so change speed
        funnelMotor.set(ControlMode.PercentOutput, withBallSpeed);
      }
    } else {
      // We are commanded off
      funnelMotor.set(ControlMode.PercentOutput, 0.0);
    }
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
