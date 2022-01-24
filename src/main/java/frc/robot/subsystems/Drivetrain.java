// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.drive.swerve.SwerveController;
import frc.lib.drive.swerve.SwerveModuleFalconFalcon;
import frc.robot.Constants;

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
  private SwerveModuleFalconFalcon m_frontLeftModule;

  // Module CAN Encoders
  private final CANCoder frontLeftEncoder;
  private final CANCoder frontRightEncoder;
  private final CANCoder rearLeftEncoder;
  private final CANCoder rearRightEncoder;

  // Turn PID Controllers
  private final PIDController frontLeftController;
  private final PIDController frontRightController;
  private final PIDController rearLeftController;
  private final PIDController rearRightController;

  // Swerve Modules
  public final SwerveModuleFalconFalcon frontLeftModule;
  public final SwerveModuleFalconFalcon frontRightModule;
  public final SwerveModuleFalconFalcon rearLeftModule;
  public final SwerveModuleFalconFalcon rearRightModule;

  // Swerve Controller
  public final SwerveController swerveController;

  // Robot Gyro
  public AHRS navx;

  // // Swerve Drive Kinematics
  // public final SwerveDriveKinematics swerveDriveKinematics;

  // // Swerve Drive Odometry
  // public final SwerveDriveOdometry swerveDriveOdometry;

  // // Swerve Pose
  // public Pose2d latestSwervePose;

  // DriveTrain Dashboard Update Counter
  private int dashboardCounter = 0;

  public Drivetrain() {
    // Drive Motors
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

    // Set motor states
    setTalonNeutralMode(NeutralMode.Coast);

    // TODO: figure out current values
    setDriveCurrentLimit(60.0);
    setTurnCurrentLimit(60.0); // potentially unused

    // Drive Encoders
    frontLeftEncoder = new CANCoder(0);
    frontLeftEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
    frontRightEncoder = new CANCoder(1);
    frontRightEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
    rearLeftEncoder = new CANCoder(2);
    rearLeftEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
    rearRightEncoder = new CANCoder(3);
    rearRightEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);

    // Turn PID Controllers
    frontLeftController = new PIDController(Constants.turnP, Constants.turnI, Constants.turnD);
    frontLeftController.enableContinuousInput(-180.0, 180.0);

    frontRightController = new PIDController(Constants.turnP, Constants.turnI, Constants.turnD);
    frontRightController.enableContinuousInput(-180.0, 180.0);

    rearLeftController = new PIDController(Constants.turnP, Constants.turnI, Constants.turnD);
    rearLeftController.enableContinuousInput(-180.0, 180.0);

    rearRightController = new PIDController(Constants.turnP, Constants.turnI, Constants.turnD);
    rearRightController.enableContinuousInput(-180.0, 180.0);

    // Set the Drive PID Controllers
    frontLeftDrive.config_kP(0, Constants.driveP);
    frontLeftDrive.config_kI(0, Constants.driveI);
    frontLeftDrive.config_kD(0, Constants.driveD);
    frontLeftDrive.config_kF(0, Constants.driveF);

    frontRightDrive.config_kP(0, Constants.driveP);
    frontRightDrive.config_kI(0, Constants.driveI);
    frontRightDrive.config_kD(0, Constants.driveD);
    frontRightDrive.config_kF(0, Constants.driveF);

    rearLeftDrive.config_kP(0, Constants.driveP);
    rearLeftDrive.config_kI(0, Constants.driveI);
    rearLeftDrive.config_kD(0, Constants.driveD);
    rearLeftDrive.config_kF(0, Constants.driveF);

    rearRightDrive.config_kP(0, Constants.driveP);
    rearRightDrive.config_kI(0, Constants.driveI);
    rearRightDrive.config_kD(0, Constants.driveD);
    rearRightDrive.config_kF(0, Constants.driveF);

    // Swerve Modules
    frontLeftModule = new SwerveModuleFalconFalcon(frontLeftDrive, frontLeftTurn, frontLeftEncoder,
        Constants.frontLeftOffset, frontLeftController, Constants.driveWheelDiameter, Constants.driveGearRatio,
        Constants.swerveMaxSpeed);

    frontRightModule = new SwerveModuleFalconFalcon(frontRightDrive, frontRightTurn, frontRightEncoder,
        Constants.frontRightOffset, frontRightController, Constants.driveWheelDiameter, Constants.driveGearRatio,
        Constants.swerveMaxSpeed);

    rearLeftModule = new SwerveModuleFalconFalcon(rearLeftDrive, rearLeftTurn, rearLeftEncoder,
        Constants.rearLeftOffset,
        rearLeftController, Constants.driveWheelDiameter, Constants.driveGearRatio, Constants.swerveMaxSpeed);

    rearRightModule = new SwerveModuleFalconFalcon(rearRightDrive, rearRightTurn, rearRightEncoder,
        Constants.rearRightOffset, rearRightController, Constants.driveWheelDiameter, Constants.driveGearRatio,
        Constants.swerveMaxSpeed);

    // Swerve Controller
    swerveController = new SwerveController(Constants.swerveLength, Constants.swerveWidth);

    // robot gyro initialization
    navx = new AHRS(SPI.Port.kMXP);

    // // Swerve Drive Kinematics
    // swerveDriveKinematics = new
    // SwerveDriveKinematics(Constants.frontLeftLocation,
    // Constants.frontRightLocation,
    // Constants.rearLeftLocation, Constants.rearRightLocation);

    // // Serve Drive Odometry
    // swerveDriveOdometry = new SwerveDriveOdometry(swerveDriveKinematics,
    // Rotation2d.fromDegrees(-navx.getYaw()),
    // new Pose2d(0.0, 0.0, new Rotation2d()));

    // Motion Trajectories
    loadMotionPaths();

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // DriveTrain Dashboard Update
    if (++dashboardCounter >= 5) {
      // Display LimeLight Distance to Target
      // SmartDashboard.putNumber("Limelight Distance",
      // limeLightCamera.getDistanceToTarget(Constants.cameraAngle,
      // Constants.cameraHeight, Constants.targetHeight));

      // Display Gyro Angle
      SmartDashboard.putNumber("Gyro Yaw", navx.getYaw());

      // Display Module Angles
      SmartDashboard.putNumber("Front Left Module Angle",
          frontLeftModule.getEncoderAngle());
      SmartDashboard.putNumber("Front Right Module Angle",
          frontRightModule.getEncoderAngle());
      SmartDashboard.putNumber("Rear Left Module Angle",
          rearLeftModule.getEncoderAngle());
      SmartDashboard.putNumber("Rear Right Module Angle",
          rearRightModule.getEncoderAngle());

      // Display Wheel Velocities
      SmartDashboard.putNumber("Front Left Module Velocity",
          frontLeftModule.getWheelSpeedMeters());
      SmartDashboard.putNumber("Front Right Module Velocity",
          frontRightModule.getWheelSpeedMeters());
      SmartDashboard.putNumber("Rear Left Module Velocity",
          rearLeftModule.getWheelSpeedMeters());
      SmartDashboard.putNumber("Rear Right Module Velocity",
          rearRightModule.getWheelSpeedMeters());

      dashboardCounter = 0;
    }

  }

  public void setTalonNeutralMode(NeutralMode mode) {
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

  public void stopDrive() {
    frontLeftModule.stop();
    frontRightModule.stop();
    rearLeftModule.stop();
    rearRightModule.stop();
  }

  // public void resetOdometry() {
  // swerveDriveOdometry.resetPosition(new Pose2d(0.0, 0.0, new Rotation2d()),
  // Rotation2d.fromDegrees(-navx.getYaw()));
  // }

  // public void setOdometryPosition(Pose2d position) {
  // swerveDriveOdometry.resetPosition(position,
  // Rotation2d.fromDegrees(-navx.getYaw()));
  // }

  private void loadMotionPaths() {
    // Trajectory Paths

  }
}