// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {

  // Drive Motors
  private WPI_TalonFX frontLeftDrive;
  private WPI_TalonFX frontLeftTurn;

  private WPI_TalonFX frontRightDrive;
  private WPI_TalonFX frontRightTurn;

  private WPI_TalonFX rearLeftDrive;
  private WPI_TalonFX rearLeftTurn;

  private WPI_TalonFX rearRightDrive;
  private WPI_TalonFX rearRightTurn;

  // Swerve modules
  private SwerveModule m_frontLeftModule;
  

  // Robot Gyro
  public AHRS navx;

  public Drivetrain() {
    //Drive Motors
    frontLeftDrive = new WPI_TalonFX(2);
    frontLeftDrive.setInverted(false);

    frontLeftTurn = new WPI_TalonFX(3);
    frontLeftTurn.setInverted(false);

    frontRightDrive = new WPI_TalonFX(4);
    frontRightDrive.setInverted(false);
    
    frontRightTurn = new WPI_TalonFX(5);
    frontRightTurn.setInverted(false);

    rearLeftDrive = new WPI_TalonFX(6);
    rearLeftDrive.setInverted(false);

    rearLeftTurn = new WPI_TalonFX(7);
    rearLeftTurn.setInverted(false);

    rearRightDrive = new WPI_TalonFX(8);
    rearRightDrive.setInverted(false);

    rearRightTurn = new WPI_TalonFX(9);
    rearRightTurn.setInverted(false);

    setTalonNeutralMode(NeutralMode.Coast);

    // TODO: figure out current values
    setDriveCurrentLimit(60.0);
    setTurnCurrentLimit(60.0); // potentially unused

    // robot gyro initialization
    navx = new AHRS(SPI.Port.kMXP);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setTalonNeutralMode(NeutralMode mode){
    frontLeftDrive.setNeutralMode(mode);
    frontLeftTurn.setNeutralMode(mode);
    frontRightDrive.setNeutralMode(mode);
    frontRightTurn.setNeutralMode(mode);
    rearLeftDrive.setNeutralMode(mode);
    rearLeftTurn.setNeutralMode(mode);
    rearRightDrive.setNeutralMode(mode);
    rearRightTurn.setNeutralMode(mode);
  }

  public void setDriveCurrentLimit(double current) {
    frontLeftDrive.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, current, current, 0));
    frontRightDrive.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, current, current, 0));
    rearLeftDrive.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, current, current, 0));
    rearRightDrive.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, current, current, 0));
  }

  // seconds from idle to max speed
  public void setDriveRampRate(double seconds) {
    // Open loop ramp rates
    frontLeftDrive.configOpenloopRamp(seconds);
    frontRightDrive.configOpenloopRamp(seconds);
    rearLeftDrive.configOpenloopRamp(seconds);
    rearRightDrive.configOpenloopRamp(seconds);
    
    // Closed loop ramp rates
    frontLeftDrive.configClosedloopRamp(seconds);
    frontRightDrive.configClosedloopRamp(seconds);
    rearLeftDrive.configClosedloopRamp(seconds);
    rearRightDrive.configClosedloopRamp(seconds);
  }

  public void setTurnCurrentLimit(double current) {
    frontLeftTurn.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, current, current, 0));
    frontRightTurn.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, current, current, 0));
    rearLeftTurn.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, current, current, 0));
    rearRightTurn.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, current, current, 0));
  }
}
