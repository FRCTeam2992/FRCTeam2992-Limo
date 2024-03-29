// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.math.MathUtil;
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

    public double turretTargetAngle = Constants.turretDefaultAngle;              // Angle the turret was last targeted to turn to
    private boolean autoAiming = false;

    public Drivetrain mDrivetrain;

    private int dashboardCounter = 0;
    public double turretTarget = 180.0;

    public Turret(Drivetrain drivetrain) {
        // Turret Motors
        turretTalon = new WPI_TalonSRX(34);
        turretTalon.setNeutralMode(NeutralMode.Brake);
        turretTalon.setInverted(true);
        turretTalon.configSelectedFeedbackSensor(FeedbackDevice.PulseWidthEncodedPosition);
        turretTalon.setStatusFramePeriod(3, 255);
        turretTalon.setStatusFramePeriod(4, 254);
        turretTalon.setStatusFramePeriod(8, 253);
        turretTalon.setStatusFramePeriod(10, 252);
        turretTalon.setStatusFramePeriod(12, 251);
        turretTalon.setStatusFramePeriod(13, 250);
        turretTalon.setStatusFramePeriod(14, 249);
        addChild("Turret Motor", turretTalon);

        // Turret PID Controller
        turretRotate = new PIDController(Constants.turretP, Constants.turretI, Constants.turretD);
        turretRotate.setTolerance(Constants.turretTolerance);
        turretRotate.disableContinuousInput();
        turretRotate.setIntegratorRange(-0.2, 0.2);


        // LimeLight Camera
        limeLightCamera = new LimeLight();

        mDrivetrain = drivetrain;
    }

    @Override
    public void periodic() {
        // Put code here to be run every loop

        if (++dashboardCounter >= 5) {

        // Update Dashboard
        SmartDashboard.putNumber("Turret Encoder", getTurretEncoder());
        SmartDashboard.putNumber("Turret Angle", angleOverlap(getTurretAngle()));
        SmartDashboard.putNumber("Turret Target", turretTargetAngle);
        // SmartDashboard.putNumber("Camera Angle", limeLightCamera.getCameraAngle(Constants.distanceTest,
        //         Constants.cameraHeight, Constants.goalHeight));
        // SmartDashboard.putNumber("Y-Offset", limeLightCamera.getTargetYOffset());
        SmartDashboard.putNumber("Distance", limeLightCamera.getDistanceToTarget(Constants.cameraAngle, Constants.cameraHeight, Constants.goalHeight));
        SmartDashboard.putNumber("x-Offset", limeLightCamera.getTargetXOffset());

        SmartDashboard.putBoolean("LL Has Target", limeLightCamera.hasTarget());
        SmartDashboard.putBoolean("Turret OnTarget", onTarget());
        SmartDashboard.putBoolean("Turret AutoAim", isAutoAiming());
        SmartDashboard.putBoolean("Turret Ready", readyToShoot());

        SmartDashboard.putNumber("Turret Raw", getTurretAngleRaw());

        dashboardCounter = 0;
        }
    }

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    public void stopTurret() {
        turretTalon.set(ControlMode.PercentOutput, 0);
    }

    public void setTurretSpeed(double speed) {
        double setSpeed = speed;

        if (setSpeed > 0 && getTurretAngleRaw() >= Constants.turretMaxSlowZone) {
            setSpeed = Math.min(0.15, setSpeed);
        }

        if (setSpeed < 0 && getTurretAngleRaw() <= Constants.turretMinSlowZone) {
            setSpeed = Math.max(-0.15, setSpeed);
        }

        if ((setSpeed > 0 && getTurretAngleRaw() >= Constants.turretMaxEnd)
                || (setSpeed < 0 && getTurretAngleRaw() < Constants.turretMinEnd)) {
            setSpeed = 0;
        }
        setSpeed = MathUtil.clamp(setSpeed, -1, 1);
        turretTalon.set(ControlMode.PercentOutput, setSpeed);
    }

    public void goToAngle(double angle) {
        turretTargetAngle = angle;          // Save the angle that was last targeted

        // SmartDashboard.putNumber("TurretToAngle Angle", angle);
        angle = angleOverlap(angle + Constants.turretRobotOffset);
        angle = Math.min(angle, Constants.turretMaxEnd);
        angle = Math.max(angle, Constants.turretMinEnd);
        
        
        if (Math.abs(angle - getTurretAngleRaw()) > Constants.turretTolerance) {
            turretRotate.setSetpoint(angle);
        }
        if (Math.abs(angle - getTurretAngleRaw()) > 5.0) {
            turretRotate.reset();
        }
        
        
        double power = turretRotate.calculate(getTurretAngleRaw());
        power = MathUtil.clamp(power, -1, 1);
        
        // SmartDashboard.putNumber("TurretToAngle Speed", power);
        
        setTurretSpeed(power);
    }

    public static double getTurretEncoder() {
        double position = -1 * (turretTalon.getSelectedSensorPosition() + Constants.turretEncoderOffset);

        while (position < 0) {
            position += 4096;
        } 
        while (position > 4096) {
            position -= 4096;
        }

        return position;
    }

    public static double getTurretAngleRaw() {
        return getTurretEncoder() * (360.0 / 4096.0);
    }
    
    public double getTurretAngle() {
        return angleOverlap(getTurretAngleRaw() - Constants.turretRobotOffset);
    }

    public static double angleOverlap(double tempAngle) {
        while (tempAngle > 360) {
            tempAngle -= 360;
        } 
        while (tempAngle < 0) {
            tempAngle += 360;
        }
        return tempAngle;
    }

    public double getGyroYaw(){
        return mDrivetrain.getGyroYaw();
    }

    public boolean onTarget(){
        return (Math.abs(turretTargetAngle - getTurretAngle()) < Constants.turretTolerance);
    }

    public boolean isAutoAiming() {
        return autoAiming;
    }

    public void setAutoAiming(boolean autoAiming) {
        this.autoAiming = autoAiming;
    }

    public void autoAimingOn() {
        this.autoAiming = true;
    }

    public void autoAimingOff() {
        this.autoAiming = false;
    }

    public boolean readyToShoot() {
        return (onTarget() || !isAutoAiming());
    } 


}