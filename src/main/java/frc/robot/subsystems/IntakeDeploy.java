// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeDeploy extends SubsystemBase {
  /** Creates a new IntakeDeploy. */
  private CANSparkMax intakeDeployMotor;

  private DigitalInput intakeLimitSwitch;

  private boolean intakeDeployedState = false;

  private int dashboardCounter = 0;

  public IntakeDeploy() {
    intakeDeployMotor = new CANSparkMax(25, MotorType.kBrushless);
    intakeDeployMotor.setIdleMode(IdleMode.kBrake);
    intakeDeployMotor.setInverted(false);

    intakeLimitSwitch = new DigitalInput(2);

    SparkMaxPIDController intakeController = intakeDeployMotor.getPIDController();
    intakeController.setOutputRange(.05, -.05);
    intakeController.setP(Constants.intakeP);
    intakeController.setI(Constants.intakeI);
    intakeController.setD(Constants.intakeD);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (++dashboardCounter >= 5) {
      SmartDashboard.putNumber("Intake Deploy Motor Encoder", getEncoderAngle());
      SmartDashboard.putBoolean("Intake Deploy Limit Switch", getLimitSwitch());

      dashboardCounter = 0;
    }

    // if (getLimitSwitch()) {
    // zeroIntakeDeployMotor();
    // }

  }

  public boolean getLimitSwitch() {
    return intakeLimitSwitch.get();
  }

  public double getEncoderAngle() {
    return intakeDeployMotor.getEncoder().getPosition();
  }

  public void deployIntake() {
    intakeDeployMotor.getPIDController().setReference(Constants.maxIntakeEncoderAngle, ControlType.kPosition);
  }

  public void retractIntake() {
    intakeDeployMotor.getPIDController().setReference(Constants.minIntakeEncoderAngle, ControlType.kPosition);
  }

  public boolean getIntakeDeployedState() {
    return intakeDeployedState;
  }

  public void setIntakeDeployedState(boolean intakeDeployedState) {
    this.intakeDeployedState = intakeDeployedState;
  }

  public void zeroIntakeDeployMotor() {
    intakeDeployMotor.getEncoder().setPosition(0.0);
  }

}
