// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.vision.LimeLight;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class Turret extends SubsystemBase {

    // Turret Motors
    private static WPI_TalonSRX turretTalon;

    // Turret PID Controller
    public PIDController turretRotate;

    // Limelight Camera
    public final LimeLight limeLightCamera;

    public double gyroYaw;

    public Drivetrain mDrivetrain;

    public Turret(Drivetrain drivetrain) {
        // Turret Motors
        turretTalon = new WPI_TalonSRX(34);
        turretTalon.setNeutralMode(NeutralMode.Brake);
        turretTalon.setInverted(true);
        turretTalon.configSelectedFeedbackSensor(FeedbackDevice.PulseWidthEncodedPosition);
        addChild("Turret Motor", turretTalon);

        // Turret PID Controller
        turretRotate = new PIDController(Constants.turretP, Constants.turretI, Constants.turretD);
        turretRotate.setTolerance(Constants.turretTolerance);
        turretRotate.disableContinuousInput();


        // LimeLight Camera
        limeLightCamera = new LimeLight();

        mDrivetrain = drivetrain;
    }

    @Override
    public void periodic() {
        // Put code here to be run every loop

        // Update Dashboard
        SmartDashboard.putNumber("Turret Encoder", getTurretPostion());
        SmartDashboard.putNumber("Turret Angle", angleOverlap(getTurretAngle()));
        SmartDashboard.putNumber("Camera Angle", limeLightCamera.getCameraAngle(Constants.distanceTest,
                Constants.cameraHeight, Constants.goalHeight));
        SmartDashboard.putNumber("Y-Offset", limeLightCamera.getTargetYOffset());
        SmartDashboard.putNumber("Distance", limeLightCamera.getDistanceToTarget(35.8, 44, 104));


        // double x = -Robot.m_robotContainer.controller0.getLeftX();
        // double y = -Robot.m_robotContainer.controller0.getLeftY();
        // double xyAngle = (Math.toDegrees(Math.atan2(y, x)) - 90);
        // SmartDashboard.putNumber("Gyro Yaw", navx.getYaw());
        // SmartDashboard.putNumber("Joystick X",
        // -Robot.m_robotContainer.controller0.getLeftX());
        // SmartDashboard.putNumber("Joystick Y",
        // -Robot.m_robotContainer.controller0.getLeftY());
        // SmartDashboard.putNumber("Joystick Angle", (angleOverlap(xyAngle)));

    }

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    public void stopTurret() {
        turretTalon.set(ControlMode.PercentOutput, 0);
    }

    public void setTurretSpeed(double speed) {
        double setSpeed = speed;

        if (setSpeed > 0 && getTurretAngle() >= Constants.turretMaxSlowZone) {
            setSpeed = Math.min(0.15, setSpeed);
        }

        if (setSpeed < 0 && getTurretAngle() <= Constants.turretMinSlowZone) {
            setSpeed = Math.max(-0.15, setSpeed);
        }

        if ((setSpeed > 0 && getTurretAngle() >= Constants.turretMaxEnd)
                || (setSpeed < 0 && getTurretAngle() < Constants.turretMinEnd)) {
            setSpeed = 0;
        }
        setSpeed = MathUtil.clamp(setSpeed, -.3, .3);
        turretTalon.set(ControlMode.PercentOutput, setSpeed);
    }

    public void goToAngle(double angle) {

        angle = angleOverlap(angle + Constants.turretRobotOffset);
        angle = Math.min(angle, Constants.turretMaxEnd);
        angle = Math.max(angle, Constants.turretMinEnd);
        
        turretRotate.setSetpoint(angle);
        double power = turretRotate.calculate(getTurretAngle());
        power = MathUtil.clamp(power, -1, 1);
        SmartDashboard.putNumber("TurretToAngle Angle", angle);
        SmartDashboard.putNumber("TurretToAngle Speed", power);
        
        setTurretSpeed(power);
    }

    public static double getTurretPostion() {
        double position = -1 * (turretTalon.getSelectedSensorPosition() + Constants.turretEncoderOffset);

        while (position < 0) {
            position += 4096;
        } 
        while (position > 4096) {
            position -= 4096;
        }

        return position;
    }

    public static double getTurretAngle() {
        return getTurretPostion() * (360.0 / 4096.0);
    }

    public double angleOverlap(double tempAngle) {
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
}