
package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;

public class Constants {

    // Shooter Constants
    public static final int defaultMainShooterSpeed = 2300;
    public static final int defaultSecondaryShooterSpeed = 1700;
    public static final int shooterEncoderPulses = 2048;

    // Drive Variables
    public static final boolean isFieldCentric = true;
    public static final boolean isVelocityControlled = true;
    public static final boolean isGyroCorrected = true;
    public static final double joystickDeadband = 0.1;
    public static double joystickXYSmoothFactor = 0.5;
    public static double joystickRotationSmoothFactor = 0.5;
    public static double joystickRotationInverseDeadband = 0.14;

    // Length and Width of the Robot in Meters (Inches: 22.0 x 24.5)
    public static final double swerveWidth = 0.591;
    public static final double swerveLength = 0.654;

    // Max Swerve Speed (Velocity Control)
    public static final double swerveMaxSpeed = 2; // (Meters per Second)

    // Swerve Wheels and Gear Ratio
    public static final double driveGearRatio = 6.75;//6.75:1
    public static final double driveWheelDiameter = 0.1016;

    // Analog Encoder Offsets (Degrees) - Opposite of Raw Reading - Bevel Gear to 
    // Right
    public static final double frontLeftOffset = 84.29;
    public static final double frontRightOffset = -144.2;
    public static final double rearLeftOffset = 101.25;
    public static final double rearRightOffset = -86.48;

    // Swerve Drive PID (Velocity Control)
    public static final double driveP = 0.05;
    public static final double driveI = 0.0;
    public static final double driveD = 0.01;
    public static final double driveF = 0.047;

    // Swerve Turn PIDs
    public static final double turnP = 0.008;
    public static final double turnI = 0.0;
    public static final double turnD = 0.00005;

    //Gyro P
    public static final double driveGyroP = 0.005;

     // Swerve Module Translations x=.591/2  y=.654/2
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
