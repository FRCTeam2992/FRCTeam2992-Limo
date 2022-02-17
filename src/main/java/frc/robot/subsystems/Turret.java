// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.vision.LimeLight;
import frc.robot.Constants;

public class Turret extends SubsystemBase {

    // Turret Motors
    private static WPI_TalonSRX turretTalon;

    // Turret PID Controller
    public PIDController turretRotate;

    // Limelight Camera
    public final LimeLight limeLightCamera;

    public Turret() {
        // Turret Motors
        turretTalon = new WPI_TalonSRX(13);
        turretTalon.setNeutralMode(NeutralMode.Brake);
        turretTalon.setInverted(true);

        // Turret PID Controller
        turretRotate = new PIDController(Constants.turretP, Constants.turretI, Constants.turretD);
        turretRotate.setTolerance(Constants.turretTolerance);
        turretRotate.disableContinuousInput();

        // LimeLight Camera
        limeLightCamera = new LimeLight();
    }

    @Override
    public void periodic() {
        // Put code here to be run every loop

        // Update Dashboard
        SmartDashboard.putNumber("Turret Angle", getTurretAngle());
        SmartDashboard.putNumber("Camera Angle", limeLightCamera.getCameraAngle(Constants.distanceTest,
                Constants.cameraHeight, Constants.goalHeight));
        SmartDashboard.putNumber("Y-Offset", limeLightCamera.getTargetYOffset());
    }

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    public void stopTurret() {
        turretTalon.set(ControlMode.PercentOutput, 0);
    }

    public void setTurretSpeed(double speed) {
        double setSpeed = speed;

        if (setSpeed > 0 && getTurretAngle() <= Constants.turretMinSlowZone) {
            setSpeed = 0.3;
        }

        if (setSpeed < 0 && getTurretAngle() >= Constants.turretMaxSlowZone) {
            setSpeed = -0.3;
        }

        if ((setSpeed > 0 && getTurretAngle() <= Constants.turretMinEnd)
                || (setSpeed < 0 && getTurretAngle() > Constants.turretMaxEnd)) {
            setSpeed = 0;
        }
        setSpeed *= 0.7;
        turretTalon.set(ControlMode.PercentOutput, setSpeed);
    }

    public void goToAngle(double angle) {
        angle = Math.min(angle, Constants.turretMaxEnd);
        angle = Math.max(angle, Constants.turretMinEnd);
        setTurretSpeed(turretRotate.calculate(getTurretAngle(), angle));
    }

    public static double getTurretPostion() {
        double position = turretTalon.getSelectedSensorPosition() - Constants.turretOffset;

        if (position < 0) {
            position += 4096;
        }

        return position;
    }

    public static double getTurretAngle() {
        return getTurretPostion() * (360.0 / 4096.0);
    }
}