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

import frc.lib.Ranging.CargoBallDataPoint;
import frc.lib.Ranging.CargoBallInterpolator;
import frc.lib.vision.LimeLight;
import frc.lib.vision.LimeLight.LedMode;
import frc.robot.commands.*;
import frc.lib.oi.controller.TriggerButton;
import frc.lib.vision.LimeLight;
import frc.lib.vision.LimeLight.LedMode;
import frc.robot.commands.*;
import frc.robot.commands.groups.AutoIntake;
import frc.robot.commands.groups.AutoShoot;
import frc.robot.commands.groups.StopAutoIntake;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

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
  // public final Intake mIntake;
  public final Drivetrain mDrivetrain;

  public final Turret mTurret;
  public final ShooterHood mShooterHood;
  public final Shooter mShooter;

  public final Intake mIntake;
  public final CargoFunnel mCargoFunnel;
  public final TopLift mTopLift;
  public final BottomLift mBottomLift;

  // Joysticks
  public XboxController controller0;
  public XboxController controller1;

  // public final LimeLight limeLightCamera;

  // A chooser for autonomous commands
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  private CargoBallInterpolator cargoBallInterpolator;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  private RobotContainer() {

    // mIntake = new Intake();
    mDrivetrain = new Drivetrain();
    mDrivetrain.setDefaultCommand(new DriveSticks(mDrivetrain));

    mTurret = new Turret(mDrivetrain);
    mTurret.setDefaultCommand(new TurretSticks(mTurret));
    //mTurret.setDefaultCommand(new TurretSticks(mTurret));
    mShooterHood = new ShooterHood();
    mShooterHood.setDefaultCommand(new StopHood(mShooterHood));
    mShooter = new Shooter();
    mShooter.setDefaultCommand(new StopShooter(mShooter));

    mIntake = new Intake();
    mIntake.setDefaultCommand(new StopIntake(mIntake));
    mCargoFunnel = new CargoFunnel();
    mCargoFunnel.setDefaultCommand(new StopCargoFunnel(mCargoFunnel));
    mTopLift = new TopLift();
    mTopLift.setDefaultCommand(new StopTopLift(mTopLift));
    mBottomLift = new BottomLift();
    mBottomLift.setDefaultCommand(new StopBottomLift(mBottomLift));

    controller0 = new XboxController(0);
    controller1 = new XboxController(1);

    // limeLightCamera = new LimeLight();

    // Smartdashboard Subsystems
    // sSmartDashboard.putData(mIntake);

    SmartDashboard.putData(mShooter);
    SmartDashboard.putData(mShooterHood);
    SmartDashboard.putData(mTurret);

    // SmartDashboard.putData(mShooter);

    SmartDashboard.putData(mCargoFunnel);

    SmartDashboard.putData(mDrivetrain);

    // SmartDashboard Buttons
    // SmartDashboard.putData("Autonomous Command", new AutonomousCommand());

    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands

    // m_chooser.setDefaultOption("Autonomous Command", new AutonomousCommand());

    // SmartDashboard.putData("Auto Mode", m_chooser);
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
    SmartDashboard.putData("Increase Main Shooter Speed", new ChangeMainShooterSpeed(mShooter, 50));
    SmartDashboard.putData("Decrease Main Shooter Speed", new ChangeMainShooterSpeed(mShooter, -50));

    SmartDashboard.putData("Increase Second Shooter Speed", new ChangeSecondaryShooterSpeed(mShooter, 50));
    SmartDashboard.putData("Decrease Second Shooter Speed", new ChangeSecondaryShooterSpeed(mShooter, -50));

    JoystickButton startShooterButton = new JoystickButton(controller1, XboxController.Button.kX.value);
    startShooterButton.toggleWhenPressed(new StartShooter(mShooter), true);

    JoystickButton moveTurretLeftButton = new JoystickButton(controller1, XboxController.Button.kLeftBumper.value);
    moveTurretLeftButton.whileHeld(new MoveTurret(mTurret, -.3), true);
    

    JoystickButton moveTurretRightButton = new JoystickButton(controller1, XboxController.Button.kRightBumper.value);
    moveTurretRightButton.whileHeld(new MoveTurret(mTurret, .3), true);

    // Temp to allow start / stop of turretsticks from Dashboard
    SmartDashboard.putData(new TurretSticks(mTurret));


    JoystickButton intakeButton = new JoystickButton(controller1, XboxController.Button.kA.value);
    intakeButton.toggleWhenPressed(new SpinIntake(mIntake, .6), true);

    JoystickButton funnelButton = new JoystickButton(controller1, XboxController.Button.kB.value);
    funnelButton.toggleWhenPressed(new SpinCargoFunnel(mCargoFunnel, .5));

    JoystickButton intakeBackButton = new JoystickButton(controller1, XboxController.Button.kY.value);
    intakeBackButton.toggleWhenPressed(new SpinIntake(mIntake, -.6), true);

    JoystickButton funnelBackButton = new JoystickButton(controller1, XboxController.Button.kY.value);
    funnelBackButton.toggleWhenPressed(new SpinCargoFunnel(mCargoFunnel, -.6));

    JoystickButton bottomLiftUpButton = new JoystickButton(controller1, XboxController.Button.kB.value);
    bottomLiftUpButton.toggleWhenPressed(new SpinBottomLift(mBottomLift, .6));

    JoystickButton bottomLiftDownButton = new JoystickButton(controller1, XboxController.Button.kY.value);
    bottomLiftDownButton.toggleWhenPressed(new SpinBottomLift(mBottomLift, -.6));

    TriggerButton topLiftUpButton = new TriggerButton(controller0, .2, 'r');
    topLiftUpButton.toggleWhenActive(new SpinTopLift(mTopLift, .6));

    JoystickButton topLiftDownButton = new JoystickButton(controller1, XboxController.Button.kY.value);
    topLiftDownButton.toggleWhenPressed(new SpinTopLift(mTopLift, -.6));

    // TriggerButton autoShoot = new TriggerButton(controller0, .2, 'r');
    // autoShoot.whenActive(new AutoShoot(mCargoFunnel, mTopLift, mBottomLift),
    // true);

    JoystickButton fieldOrientButton = new JoystickButton(controller0, XboxController.Button.kStart.value);
    fieldOrientButton.whenPressed(new ResetGyro(mDrivetrain), true);

    POVButton moveHoodUpButton = new POVButton(controller0, 0);
    moveHoodUpButton.whenPressed(new MoveHood(mShooterHood, .5), true);
    moveHoodUpButton.whenReleased(new StopHood(mShooterHood), true);

    POVButton moveHoodDownButton = new POVButton(controller0, 180);
    moveHoodDownButton.whenPressed(new MoveHood(mShooterHood, -.5), true);
    moveHoodDownButton.whenReleased(new StopHood(mShooterHood), true);

    SmartDashboard.putData("0 Wheels", new SetSwerveAngle(mDrivetrain, 0, 0, 0, 0));

    SmartDashboard.putData("0 Hood", new MoveHoodToAngle(mShooterHood, 0.0));
    SmartDashboard.putData("Top Hood", new MoveHoodToAngle(mShooterHood, 140.0));
    SmartDashboard.putData("Bottom Hood", new MoveHoodToAngle(mShooterHood, -140.0));
    // SmartDashboard.putData("100 Hood", new MoveHoodToAngle(mShooterHood,
    // -140.0));
    // SmartDashboard.putData("120 Hood", new MoveHoodToAngle(mShooterHood,
    // -140.0));
    // SmartDashboard.putData("130 Hood", new MoveHoodToAngle(mShooterHood,
    // -140.0));

    SmartDashboard.putData("turret forward", new MoveTurretToAngle(mTurret, 180, 1));
    SmartDashboard.putData("turret backward", new MoveTurretToAngle(mTurret, 270, 1));
    SmartDashboard.putData("turret 45", new MoveTurretToAngle(mTurret, 90, 1));

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

  public void initInterpolator() {
    cargoBallInterpolator = new CargoBallInterpolator();

    // Example adding point
    // cargoBallInterpolator.addDataPoint(new CargoBallDataPoint(0.0, 0, 0, 0.0));
  }

}
