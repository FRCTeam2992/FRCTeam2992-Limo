// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.vision.LimeLight;
import frc.robot.Constants;
import frc.robot.Robot;

public class Turret extends SubsystemBase {

    // Turret Motors
    private static WPI_TalonFX turretTalon;
    private static CANCoder turretEncoder;

    // Turret PID Controller
    public PIDController turretRotate;
    double pidPower = 0.0;              // Last commanded PID power

    // Limelight Camera
    public final LimeLight limeLightCamera;

    public double turretTargetAngle = Constants.turretDefaultAngle;              // Angle the turret was last targeted to turn to
    private boolean autoAiming = false;

    public Drivetrain mDrivetrain;

    private int dashboardCounter = 0;
    public double turretTarget = 180.0;

    public Pose2d limeLightPose = new Pose2d();
    public Pose2d turretPose = new Pose2d();
    public Pose2d visionPose = new Pose2d();


    public Turret(Drivetrain drivetrain) {
        // Turret Motors
        turretTalon = new WPI_TalonFX(34, "CanBus2");
        turretTalon.setNeutralMode(NeutralMode.Brake);
        turretTalon.setInverted(false);
        //turretTalon.configSelectedFeedbackSensor(FeedbackDevice.PulseWidthEncodedPosition);
        turretTalon.setStatusFramePeriod(3, 255);
        turretTalon.setStatusFramePeriod(4, 254);
        turretTalon.setStatusFramePeriod(8, 253);
        turretTalon.setStatusFramePeriod(10, 252);
        turretTalon.setStatusFramePeriod(12, 251);
        turretTalon.setStatusFramePeriod(13, 250);
        turretTalon.setStatusFramePeriod(14, 249);
        addChild("Turret Motor", turretTalon);

        turretEncoder = new CANCoder(34, "CanBus2");
        turretEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);

        // Turret PID Controller
        turretRotate = new PIDController(Constants.turretP, Constants.turretI, Constants.turretD);
        turretRotate.setTolerance(Constants.turretTolerance);
        turretRotate.disableContinuousInput();
        turretRotate.setIntegratorRange(-0.18, 0.14);


        // LimeLight Camera
        limeLightCamera = new LimeLight();

        mDrivetrain = drivetrain;
    }

    @Override
    public void periodic() {
        // Put code here to be run every loop

    
       if (limeLightCamera.hasTarget()) {
            // Update limelight sighted pose -- based on last seen target and may be STALE
            limeLightPose = calcLLPose(limeLightPose);       // Unchanged if no current target seen
            turretPose = calcTurretPose(limeLightPose);      // This is the pose of center of turret
            visionPose = calcRobotPose(turretPose);          // This is the pose of the drive chassis

            // Update pose estimator based on target
            Robot.mRobotContainer.mDrivetrain.swerveDrivePoseEstimator.addVisionMeasurement(visionPose, 
                    Timer.getFPGATimestamp());
       }

        if (++dashboardCounter >= 5) {

        // Update Dashboard
        SmartDashboard.putNumber("Turret Encoder", getTurretEncoder());
        SmartDashboard.putNumber("Turret Angle Raw", getTurretAngleRaw());
        SmartDashboard.putNumber("Turret Angle", angleOverlap(getTurretAngle()));
        // SmartDashboard.putNumber("Turret Target", turretTargetAngle);
        // SmartDashboard.putNumber("Camera Angle", limeLightCamera.getCameraAngle(Constants.distanceTest,
        //         Constants.cameraHeight, Constants.goalHeight));
        // SmartDashboard.putNumber("Y-Offset", limeLightCamera.getTargetYOffset());
        // SmartDashboard.putNumber("Distance", limeLightCamera.getDistanceToTarget(Constants.cameraAngle, Constants.cameraHeight, Constants.goalHeight));
        // SmartDashboard.putNumber("x-Offset", limeLightCamera.getTargetXOffset());

        // SmartDashboard.putBoolean("LL Has Target", limeLightCamera.hasTarget());
        // SmartDashboard.putBoolean("Turret OnTarget", onTarget());
        // SmartDashboard.putBoolean("Turret AutoAim", isAutoAiming());
        // SmartDashboard.putBoolean("Turret Ready", readyToShoot());

        // SmartDashboard.putNumber("Turret Raw", getTurretAngleRaw());

        // SmartDashboard.putNumber("LL Pose X", limeLightPose.getX() * 100 / 2.54);
        // SmartDashboard.putNumber("LL Pose Y", limeLightPose.getY() * 100 / 2.54);
        // SmartDashboard.putNumber("LL Pose Angle", limeLightPose.getRotation().getDegrees());

        // SmartDashboard.putNumber("Turret Pose X", turretPose.getX() * 100 / 2.54);
        // SmartDashboard.putNumber("Turret Pose Y", turretPose.getY() * 100 / 2.54);
        // SmartDashboard.putNumber("Turret Pose Angle", turretPose.getRotation().getDegrees());

        // SmartDashboard.putNumber("Vision Pose X", visionPose.getX() * 100 / 2.54);
        // SmartDashboard.putNumber("Vision Pose Y", visionPose.getY() * 100 / 2.54);
        // SmartDashboard.putNumber("Vision Pose Angle", visionPose.getRotation().getDegrees());

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
            setSpeed = Math.min(0.10, setSpeed);
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
        if (Math.abs(angle - getTurretAngleRaw()) > 20.0) {
            turretRotate.reset();
        }
        
        // power = 0.0;
        pidPower = turretRotate.calculate(getTurretAngleRaw());
        pidPower += Constants.turretF;
    
        pidPower = MathUtil.clamp(pidPower, -.50, 0.46);
        
        SmartDashboard.putNumber("TurretToAngle Speed", pidPower);
        
        setTurretSpeed(pidPower);
    }

    public static double getTurretEncoder() {
        double position = (turretEncoder.getAbsolutePosition() + Constants.turretEncoderOffset);

        while (position < 0) {
            position += 360.0;
        } 
        while (position > 360.0) {
            position -= 360.0;
        }

        return position;
    }

    public static double getTurretAngleRaw() {
        return angleOverlap(getTurretEncoder() * 2.3);       // Adjust for gear ratio of abs encoder
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
        return ((Math.abs(turretTargetAngle - getTurretAngle()) < Constants.turretTolerance) &&
            (Math.abs(pidPower) < 0.02));
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


    public Pose2d calcLLPose (Pose2d defaultPose) { 
        Pose2d tempPose = new Pose2d(0.0, 0.0, new Rotation2d(0.0));

        if (limeLightCamera.hasTarget()) {
            // Calculate the vector to goal relative to shooter turret
            double distance = limeLightCamera.getDistanceToTarget(Constants.cameraAngle, 
                Constants.cameraHeight, Constants.goalHeight);
            // Convert to real world distance using interpolater and data table
            distance = Robot.mRobotContainer.cargoBallInterpolator.calcRealDistance(distance);

            // Convert to meters and Add radius of the goal -- distance from LL to center of field
            distance *= 2.54 / 100;     // Now in meters
            distance += Constants.goalRadius;

            // Now calculate the angle adjusting for turret position and field oriented
            double llAngle = getTurretAngle() + getGyroYaw();
            double angle = llAngle + limeLightCamera.getTargetXOffset();
            
            // Calculate translation from center of hub to limelight
            Rotation2d targetAngle = Rotation2d.fromDegrees(180-angle);
            Translation2d limelightVector = new Translation2d(distance, targetAngle);
            Transform2d limelightTransform = new Transform2d(limelightVector, Rotation2d.fromDegrees(llAngle)); 

            // Calculate the pose of the limelight camera
            tempPose = Constants.goalPose.plus(limelightTransform);
        } else {
            // Not seeing the target so return the default
            return tempPose = defaultPose;
        }

        return tempPose;
    }

    public Pose2d calcTurretPose(Pose2d llPose) {
        Pose2d tempPose = llPose;

        Translation2d llOffset = new Translation2d (Constants.limeLightOffset, 
               Rotation2d.fromDegrees(180 - getTurretAngle() - getGyroYaw()));
        Transform2d llTransform = new Transform2d(llOffset, Rotation2d.fromDegrees(0.0));

        return tempPose.plus(llTransform);
    }

    public Pose2d calcRobotPose(Pose2d turretPose) {
        Pose2d tempPose = turretPose;

        Translation2d turretOffset = new Translation2d(Constants.turretOffset, 
            Rotation2d.fromDegrees(180 - getGyroYaw()));
        Transform2d turretTransform = new Transform2d(turretOffset, Rotation2d.fromDegrees(-getTurretAngle()));

        return tempPose.plus(turretTransform);
    }

}