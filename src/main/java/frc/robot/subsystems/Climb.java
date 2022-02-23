// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ResourceBundle.Control;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climb extends SubsystemBase {

  private Solenoid climbSolenoid;
  private Solenoid climbSationaryHookSolenoid;

  private WPI_TalonFX climbRightMotor;
  private WPI_TalonFX climbLeftMotor;

  public boolean toggleClimbMode = false;

  /** Creates a new Climb. */
  public Climb() {
    climbRightMotor = new WPI_TalonFX(20);
    climbRightMotor.setNeutralMode(NeutralMode.Brake);
    addChild("climb right motor", climbRightMotor);

    climbLeftMotor = new WPI_TalonFX(21);
    climbLeftMotor.setNeutralMode(NeutralMode.Brake);
    addChild("climb left motor", climbLeftMotor);

    climbSolenoid = new Solenoid(PneumaticsModuleType.REVPH , 3);
    addChild("climb solenoid", climbSolenoid);

    climbSationaryHookSolenoid = new Solenoid(PneumaticsModuleType.REVPH , 4);
    addChild("stationary climb solenoid", climbSationaryHookSolenoid);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void deployClimb(boolean toggle){
    climbSolenoid.set(toggle);
  }

  public void deployStationaryHook(boolean toggle){
    climbSationaryHookSolenoid.set(toggle);
  }

  public void moveClimb(double speed){
    if (toggleClimbMode){
      climbRightMotor.set(ControlMode.PercentOutput, speed);
      climbLeftMotor.set(ControlMode.PercentOutput, speed);
    }
    else {
      climbRightMotor.set(ControlMode.PercentOutput, 0);
      climbLeftMotor.set(ControlMode.PercentOutput, 0);
    }
  }
}
