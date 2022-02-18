// RobotBuilder Version: 4.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

// ROBOTBUILDER TYPE: RobotContainer.

package frc.robot;

import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.subsystems.*;

// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot
 * (including subsystems, commands, and button mappings) should be declared
 * here.
 */
public class RobotContainer {

  private static RobotContainer m_robotContainer = new RobotContainer();

  // The robot's subsystems
  public final Intake mIntake;

  public final Shooter mShooter;

  public final CargoFunnel mCargoFunnel;

  public final Drivetrain mDrivetrain;
  
  // Joysticks
  public final XboxController controller0 = new XboxController(0);

  // A chooser for autonomous commands
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  private RobotContainer() {

    mIntake = new Intake();

    mShooter = new Shooter();
    mShooter.setDefaultCommand(new StopShooter(mShooter));

    mCargoFunnel = new CargoFunnel();
    mCargoFunnel.setDefaultCommand(new StopCargoFunnel(mCargoFunnel));
    mDrivetrain = new Drivetrain();
    mDrivetrain.setDefaultCommand(new DriveSticks(mDrivetrain));

    // Smartdashboard Subsystems
    SmartDashboard.putData(mIntake);

    SmartDashboard.putData(mShooter);

    SmartDashboard.putData(mCargoFunnel);

    SmartDashboard.putData(mDrivetrain);

    // SmartDashboard Buttons
    SmartDashboard.putData("Autonomous Command", new AutonomousCommand());

    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands

    m_chooser.setDefaultOption("Autonomous Command", new AutonomousCommand());

    SmartDashboard.putData("Auto Mode", m_chooser);
  }

  public static RobotContainer getInstance() {
    return m_robotContainer;
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Create some buttons
    // final POVButton decreaseShooterSpeed = new POVButton(controller0, 180);
    // decreaseShooterSpeed.whenPressed(new ChangeMainShooterSpeed(mShooter, -100), true);
    SmartDashboard.putData("DecreaseShooterSpeed", new ChangeMainShooterSpeed(mShooter, -100));

    // final POVButton increaseShooterSpeed = new POVButton(controller0, 0);
    // increaseShooterSpeed.whenPressed(new ChangeMainShooterSpeed(mShooter, 100), true);
    SmartDashboard.putData("increaseShooterSpeed", new ChangeMainShooterSpeed(mShooter, 100));

    final JoystickButton startShooterButton = new JoystickButton(controller0, XboxController.Button.kX.value);
    startShooterButton.toggleWhenPressed(new StartShooter(mShooter), true);
    //SmartDashboard.putData("Start Shooter", new StartShooter(mShooter));

    // final JoystickButton increaseSecondSpeed = new JoystickButton(controller0, XboxController.Button.kY.value);
    // increaseSecondSpeed.whenPressed(new ChangeSecondaryShooterSpeed(mShooter, 50), true);
    SmartDashboard.putData("IncreaseSecondaryShooter", new ChangeSecondaryShooterSpeed(mShooter, 50));

  //   final JoystickButton decreaseSecondSpeed = new JoystickButton(controller0, XboxController.Button.kA.value);
  //   decreaseSecondSpeed.whenPressed(new ChangeSecondaryShooterSpeed(mShooter, -50), true);
   // SmartDashboard.putData("DecreaseSecondaryShooter", new ChangeSecondaryShooterSpeed(mShooter, -50));

    // final JoystickButton startIntakeButton = new JoystickButton(controller0, XboxController.Button.kLeftBumper.value);
    // startIntakeButton.toggleWhenPressed(new StartIntake(mIntake, 0.75));
    // SmartDashboard.putData("Start Intake", new StartIntake(mIntake, 0.75));

    // final JoystickButton deployIntakeButton = new JoystickButton(controller0, XboxController.Button.kRightBumper.value);
    // deployIntakeButton.whenPressed(new DeployIntake(mIntake));
    // SmartDashboard.putData("Deploy Intake", new DeployIntake(mIntake));

    SmartDashboard.putData("0 Modules", new SetSwerveAngle(mDrivetrain, 0.0, 0.0, 0.0, 0.0));
    SmartDashboard.putData("180 Modules", new SetSwerveAngle(mDrivetrain, 180.0, 180.0, 180.0, 180.0));


    final JoystickButton swerveXPatterButton = new JoystickButton(controller0, XboxController.Button.kX.value);
    swerveXPatterButton.whenPressed(new SetSwerveAngle(mDrivetrain, 45.0, -45.0, -45.0, 45.0));
    swerveXPatterButton.whenReleased(new DriveSticks(mDrivetrain));

    SmartDashboard.putData("X Pattern", new SetSwerveAngle(mDrivetrain, 45.0, -45.0, -45.0, 45.0));

    

    // SmartDashboard.putData("90 Modules", new SetSwerveAngle(mDrivetrain, 90));
    // SmartDashboard.putData("180 Modules", new SetSwerveAngle(mDrivetrain, 180));
    // SmartDashboard.putData("270 Modules", new SetSwerveAngle(mDrivetrain, -90));

    final JoystickButton resetGyroButton = new JoystickButton(controller0, XboxController.Button.kStart.value);
    resetGyroButton.whenPressed(new ResetGyro(mDrivetrain));

    final JoystickButton cargoFunnelButton = new JoystickButton(controller0, XboxController.Button.kLeftBumper.value);
    cargoFunnelButton.whenPressed(new SpinCargoFunnel(mCargoFunnel, .5));
    cargoFunnelButton.whenReleased(new StopCargoFunnel(mCargoFunnel));

    final JoystickButton cargoFunnelButton2 = new JoystickButton(controller0, XboxController.Button.kRightBumper.value);
    cargoFunnelButton2.toggleWhenPressed(new SpinCargoFunnel(mCargoFunnel, .5));

    SmartDashboard.putData("spin cargo funnel", new SpinCargoFunnel(mCargoFunnel, .5));;

  }

  // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=FUNCTIONS
  public XboxController getController0() {
    return controller0;
  }

  // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=FUNCTIONS

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // The selected command will be run in autonomous
    return m_chooser.getSelected();
  }

}
