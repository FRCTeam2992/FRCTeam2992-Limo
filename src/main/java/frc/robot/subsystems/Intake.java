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

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

/**
 *
 */
public class Intake extends SubsystemBase {


    private WPI_TalonFX intakeMotor;
   
    private boolean intakeDeployed = false;

    private int dashboardCounter = 0;

    private boolean intakeCommanded = false;                 // Did driver request intake running
    private double speedCommanded = 0;                       // Requested speed

    public Intake() {

        intakeMotor = new WPI_TalonFX(21);
        intakeMotor.setNeutralMode(NeutralMode.Brake);
        intakeMotor.setInverted(false);
        intakeMotor.setStatusFramePeriod(1, 255);
        intakeMotor.setStatusFramePeriod(2, 254);
        intakeMotor.setStatusFramePeriod(3, 253);
        intakeMotor.setStatusFramePeriod(4, 252);
        intakeMotor.setStatusFramePeriod(8, 251);
        intakeMotor.setStatusFramePeriod(10, 250);
        intakeMotor.setStatusFramePeriod(12, 249);
        intakeMotor.setStatusFramePeriod(13, 248);
        intakeMotor.setStatusFramePeriod(14, 247);
        intakeMotor.setStatusFramePeriod(21, 246);
        addChild("Intake Motor", intakeMotor);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        if (++dashboardCounter >= 5) {
            // SmartDashboard.putNumber("Intake Motor Speed", speedCommanded);
            // SmartDashboard.putBoolean("Intake Commanded", intakeCommanded);
            dashboardCounter = 0;
        }

    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run when in simulation

    }

    // Put methods for controlling this subsystem
    public void deployIntake(boolean toggle) {
        // TODO:  Figure out code to deploy/retract intake
        intakeDeployed = toggle;
    }

    public boolean getIntakePosition() {
        return intakeDeployed;
    }

    public void setIntakeSpeed(double speed) {
        intakeMotor.set(ControlMode.PercentOutput, speed);
    }

    public void setIntakeCommanded (boolean commanded) {
        intakeCommanded = commanded;
    }

    public boolean getIntakeCommanded () {
        return intakeCommanded;
    }

    public double getSpeedCommanded() {
        return speedCommanded;
    }

    public void setSpeedCommanded(double speedCommanded) {
        this.speedCommanded = speedCommanded;
    }
}
