
package frc.robot;

// import org.ejml.FancyPrint;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be
 * declared globally (i.e. public static). Do not put anything functional in
 * this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
import edu.wpi.first.math.geometry.Translation2d;

public class Constants {

  // Shooter Constants
  public static final int defaultMainShooterSpeed = 1700;
  public static final int defaultSecondaryShooterSpeed = 2300;
  public static final int shooterEncoderPulses = 2048;

  // Intake Constant
  public static final double defaultInatkeSpeed = 0.5;

  // Turret Constants
  public static final double turretP = 0.062;
  public static final double turretI = 0.0;
  public static final double turretD = 0.003;
  public static final double turretTolerance = 0.5;
  public static final int turretOffset = 2074;
  public static final int turretMinEnd = 20;
  public static final int turretMaxEnd = 340;
  public static final int turretMaxSlowZone = 300;
  public static final int turretMinSlowZone = 60;
  public static final double turretJoystickDeadband = .15;

  // Vision Constants

  // the tooth to tooth of the hood
  public static final double hoodAngleRatio = 34.000 / 536.000;
  // the max and min of the hood angle not the encoder values
  public static final double minHoodPosition = -160.0;
  public static final double minHoodPZone = -125.0;

  public static final double maxHoodPosition = 165.0;
  public static final double maxHoodPZone = 130.0;

  public static final double hoodPValueBottom = 0.012;
  public static final double hoodPValueTop = 0.01515;
  public static final double hoodEncoderOffset = -(-100.7 + 170);

  public static final double cameraHeight = 44.5;
  public static final double goalHeight = 103.12;
  public static final double distanceTest = 120;
  public static final double cameraAngle = 0;

  public static final double hoodP = 0;
  public static final double hoodI = 0;
  public static final double hoodD = 0;

  public static final double hoodTolerance = 0;

  // Drive Variables
  public static final boolean isFieldCentric = true;
  public static final boolean isVelocityControlled = true;
  public static final boolean isGyroCorrected = true;
  public static final double joystickDeadband = 0.15;
  public static double joystickXYSmoothFactor = 0.5;
  public static double joystickRotationSmoothFactor = 0.5;
  public static double joystickRotationInverseDeadband = 0.14;

  // Length and Width of the Robot in Meters (Inches: 22.0 x 24.5)
  public static final double swerveWidth = 0.591;
  public static final double swerveLength = 0.654;

  // Max Swerve Speed (Velocity Control)
  public static final double swerveMaxSpeed = 3; // (Meters per Second)

  // Swerve Wheels and Gear Ratio
  public static final double driveGearRatio = 6.75;// 6.75:1
  public static final double driveWheelDiameter = 0.1016;

  // Analog Encoder Offsets (Degrees) - Opposite of Raw Reading - Bevel Gear to
  // Right
  public static final double frontLeftOffset = 147.2;
  public static final double frontRightOffset = -144.8;
  public static final double rearLeftOffset = 101.3;
  public static final double rearRightOffset = -24.2;

  // Swerve Drive PID (Velocity Control)
  public static final double driveP = 0.05;
  public static final double driveI = 0.0;
  public static final double driveD = 0.01;
  public static final double driveF = 0.047;

  // Swerve Turn PIDs
  public static final double turnP = .013;
  public static final double turnI = 0.0;
  public static final double turnD = 0.0000;

  // Gyro P
  public static final double driveGyroP = 0.005;

  // Swerve Module Translations x=.591/2 y=.654/2
  public static final Translation2d frontLeftLocation = new Translation2d(0.2955, 0.327);
  public static final Translation2d frontRightLocation = new Translation2d(0.2955, -0.327);
  public static final Translation2d rearLeftLocation = new Translation2d(-0.2955, 0.327);
  public static final Translation2d rearRightLocation = new Translation2d(-0.2955, -0.327);

  // Swerve X Axis Correction PID (Path Following)
  public static final double xCorrectionP = 5.0;
  public static final double xCorrectionI = 0.0;
  public static final double xCorrectionD = 0.0;

  // Swerve Y Axis Correction PID (Path Following)
  public static final double yCorrectionP = 5.0;
  public static final double yCorrectionI = 0.0;
  public static final double yCorrectionD = 0.0;

  // Swerve Theta Axis Correction PID (Path Following)
  public static final double thetaCorrectionP = 6.0;
  public static final double thetaCorrectionI = 0.0;
  public static final double thetaCorrectionD = 0.0;

  // Max Path Following Drive Speeds
  public static final double maxPathFollowingVelocity = 2.0; // (Meters per Second)
  public static final double maxPathFollowingAcceleration = 0.5; // (Meters per Second Squared)

  // Max Path Following Turn Speeds
  public static final double maxThetaVelocity = 6.28; // (Radians per Second)
  public static final double maxThetaAcceleration = 6.28; // (Radians per Second Squared)

}
