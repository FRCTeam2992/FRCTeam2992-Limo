// RobotBuilder Version: 4.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

// ROBOTBUILDER TYPE: Subsystem.

package frc.robot.subsystems;

import frc.robot.commands.*;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

/**
 *
 */
public class Intake extends SubsystemBase {

    private WPI_TalonSRX intakeMotor;
    private Solenoid intakeSolenoid;

    private boolean intakeDeployed = false;

    public Intake() {

        intakeMotor = new WPI_TalonSRX(11);
        intakeMotor.setNeutralMode(NeutralMode.Brake);
        intakeMotor.setInverted(true);
        addChild("Intake Motor", intakeMotor);

        intakeSolenoid = new Solenoid(PneumaticsModuleType.REVPH, 0);
        addChild("Intake Solenoid", intakeSolenoid);

        setDefaultCommand(new StopIntake(this));
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run when in simulation

    }

    // Put methods for controlling this subsystem
    public void deployIntake(boolean toggle) {
        intakeSolenoid.set(toggle);
        intakeDeployed = toggle;
    }

    public boolean getIntakeSloenoid(){
        return intakeSolenoid.get();
    }

    public void setIntakeMotor(double powerLevel){
        intakeMotor.set(TalonSRXControlMode.PercentOutput, powerLevel);
    }

}
