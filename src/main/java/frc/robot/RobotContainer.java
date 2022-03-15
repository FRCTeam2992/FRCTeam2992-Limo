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
import frc.robot.commands.*;
import frc.robot.commands.Autonomous.AutoP3S1M;
import frc.robot.commands.Autonomous.FiveBallAuto;
import frc.robot.commands.Autonomous.ThreeBallAuto;
import frc.robot.commands.Autonomous.TwoBallAuto;
import frc.lib.oi.controller.TriggerButton;
import frc.robot.commands.groups.AutoIntake;
import frc.robot.commands.groups.AutoShoot;
import frc.robot.commands.groups.DejamBallPath;
import frc.robot.commands.groups.PanicIntake;
import frc.robot.commands.groups.StopAutoIntake;
import frc.robot.paths.StraightPath;
import frc.robot.paths.TestPath;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.wpilibj.XboxController;
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

  private SendableChooser<Command> autoChooser;

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
  public final IntakeDeploy mIntakeDeploy;

  // Joysticks
  public XboxController controller0;
  public XboxController controller1;

  //Cameras
  public UsbCamera intakeCamera;
  public MjpegServer virtualCamera;

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
    
    mShooterHood = new ShooterHood();
    mShooterHood.setDefaultCommand(new HoldHoodAngle(mShooterHood));
    
    mShooter = new Shooter();
    mShooter.setDefaultCommand(new DefaultShooter(mShooter));
    
    mTopLift = new TopLift();
    mTopLift.setDefaultCommand(new DefaultTopLift(mTopLift));
    
    mBottomLift = new BottomLift();
    mBottomLift.setDefaultCommand(new DefaultBottomLift(mBottomLift));

    mCargoFunnel = new CargoFunnel(mBottomLift);
    mCargoFunnel.setDefaultCommand(new DefaultCargoFunnel(mCargoFunnel));
    
    mIntake = new Intake();
    mIntake.setDefaultCommand(new DefaultIntake(mIntake));

    mIntakeDeploy = new IntakeDeploy();
    mIntakeDeploy.setDefaultCommand(new DefaultIntakeDeploy(mIntakeDeploy));
    
    controller0 = new XboxController(0);
    controller1 = new XboxController(1);

    initInterpolator();
    setupAutoSelector();

    // limeLightCamera = new LimeLight();

    // Smartdashboard Subsystems
    SmartDashboard.putData(mShooter);
    SmartDashboard.putData(mShooterHood);
    SmartDashboard.putData(mTurret);
    SmartDashboard.putData(mIntake);
    SmartDashboard.putData(mCargoFunnel);
    SmartDashboard.putData(mDrivetrain);
    SmartDashboard.putData(mBottomLift);
    SmartDashboard.putData(mTopLift);
    SmartDashboard.putData(mIntakeDeploy);

    // SmartDashboard Buttons
    // SmartDashboard.putData("Autonomous Command", new AutonomousCommand());

    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands

    // m_chooser.setDefaultOption("Autonomous Command", new AutonomousCommand());

    // SmartDashboard.putData("Auto Mode", m_chooser);

    //Initialize cameras
    initCamera();
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

  /*
  controller0 buttons
  */
    //-Triggers
      TriggerButton autoAimButton = new TriggerButton(controller0, .4, 'l');
      autoAimButton.whileActiveContinuous(new AutoTurretAim(mTurret));
      autoAimButton.whileActiveContinuous(new AutoLimelightHood(mTurret, mShooterHood, cargoBallInterpolator));
      autoAimButton.whileActiveContinuous(new AutoLimelightMainShooter(mTurret, mShooter, cargoBallInterpolator));
      autoAimButton.whileActiveContinuous(new AutoLimelightSecondShooter(mTurret, mShooter, cargoBallInterpolator));

      TriggerButton autoShootButton = new TriggerButton(controller0, .4, 'r');
      autoShootButton.whileActiveContinuous(new AutoShoot(mCargoFunnel, mTopLift, mBottomLift,
            mShooter, mShooterHood, mTurret, mDrivetrain), true);
      autoShootButton.whileActiveContinuous(new SetSwerveAngleSafe(mDrivetrain, 45, -45, -45, 45));


      JoystickButton panicIntakeButton1 = new JoystickButton(controller0, XboxController.Button.kRightBumper.value);
      panicIntakeButton1.whileHeld(new PanicIntake(mIntake, mIntakeDeploy));

      JoystickButton intakeOrientCameraButton = new JoystickButton(controller0, XboxController.Button.kLeftBumper.value);

    //-D-Pad
      POVButton xPatternButtonUp = new POVButton(controller0, 0);
      xPatternButtonUp.whenHeld(new SetSwerveAngleSafe(mDrivetrain, 45, -45, -45, 45));
      POVButton xPatternButtonRight = new POVButton(controller0, 90);
      xPatternButtonRight.whenHeld(new SetSwerveAngleSafe(mDrivetrain, 45, -45, -45, 45));
      POVButton xPatternButtonDown = new POVButton(controller0, 180);
      xPatternButtonDown.whenHeld(new SetSwerveAngleSafe(mDrivetrain, 45, -45, -45, 45));
      POVButton xPatternButtonLeft = new POVButton(controller0, 270);
      xPatternButtonLeft.whenHeld(new SetSwerveAngleSafe(mDrivetrain, 45, -45, -45, 45));

    //-ABXY
      JoystickButton increaseMainShooterSpeed = new JoystickButton(controller0, XboxController.Button.kY.value);
      increaseMainShooterSpeed.whenPressed(new ChangeMainShooterSpeed(mShooter, 50));

      JoystickButton decreaseMainShooterSpeed = new JoystickButton(controller0, XboxController.Button.kB.value);
      decreaseMainShooterSpeed.whenPressed(new ChangeMainShooterSpeed(mShooter, -50));

      JoystickButton increaseSecondShooterSpeed = new JoystickButton(controller0, XboxController.Button.kX.value);
      increaseSecondShooterSpeed.whenPressed(new ChangeSecondaryShooterSpeed(mShooter, 50));

      JoystickButton decreaseSecondShooterSpeed = new JoystickButton(controller0, XboxController.Button.kA.value);
      decreaseSecondShooterSpeed.whenPressed(new ChangeSecondaryShooterSpeed(mShooter, -50));

    //-Other Buttons
      JoystickButton fieldOrientButton = new JoystickButton(controller0, XboxController.Button.kStart.value);
      fieldOrientButton.whenPressed(new ResetGyro(mDrivetrain), true);

      //TODO: Remove Field Orient
    


  /*
  controller1 Buttons
  */
    //-Triggers and Bumpers
      TriggerButton dejamButton = new TriggerButton(controller1, .3, 'r');
      dejamButton.whileActiveContinuous(new DejamBallPath(mIntake, mCargoFunnel, mBottomLift, mTopLift, mIntakeDeploy), true);

      TriggerButton lowGoalShotButton = new TriggerButton(controller1, .3, 'l');
      lowGoalShotButton.whileActiveOnce(new NewHoodTarget(mShooterHood, 152), true);
      lowGoalShotButton.whileActiveOnce(new SetShooterSpeedTargets(mShooter, 1200, 0), true);
      lowGoalShotButton.whileActiveOnce(new SetTurretTargetAngle(mTurret, true, 180));

      JoystickButton highGoalShotButton = new JoystickButton(controller1, XboxController.Button.kLeftBumper.value);
      highGoalShotButton.whenHeld(new NewHoodTarget(mShooterHood, -152));
      highGoalShotButton.whenHeld(new SetShooterSpeedTargets(mShooter, 1750, 2650));
      highGoalShotButton.whenHeld(new SetTurretTargetAngle(mTurret, true, 0.0));
      
    //-D-Pad
      POVButton moveHoodUpButton = new POVButton(controller1, 0);
      moveHoodUpButton.whileHeld(new MoveHood(mShooterHood, .25), true);

      POVButton moveHoodDownButton = new POVButton(controller1, 180);
      moveHoodDownButton.whileHeld(new MoveHood(mShooterHood, -.25), true);

      POVButton moveTurretLeftButton = new POVButton(controller1, 270);
      moveTurretLeftButton.whileHeld(new MoveTurret(mTurret, -.45), true);

      POVButton moveTurretRightButton = new POVButton(controller1, 90);
      moveTurretRightButton.whileHeld(new MoveTurret(mTurret, .45), true);


    //-ABXY
      JoystickButton startShooterButton = new JoystickButton(controller1, XboxController.Button.kX.value);
      startShooterButton.whenPressed(new SetShooterCommanded(mShooter, true), true);

      JoystickButton autoIntakeButton = new JoystickButton(controller1, XboxController.Button.kA.value);
      autoIntakeButton.whenPressed(new AutoIntake(mIntake, mCargoFunnel, mBottomLift, mTopLift, mIntakeDeploy), true);
      // autoIntakeButton.whenPressed(new ChangeIntakeState(mIntakeDeploy, true), true);

      JoystickButton stopAutoIntakeButton = new JoystickButton(controller1, XboxController.Button.kB.value);
      stopAutoIntakeButton.whenPressed(new StopAutoIntake(mIntake, mCargoFunnel, mBottomLift, mTopLift, mIntakeDeploy), true);


    
      //-Other Buttons
      
      JoystickButton stopShooterButton = new JoystickButton(controller1, XboxController.Button.kBack.value);
      stopShooterButton.whenPressed(new SetShooterCommanded(mShooter, false), true);

      //TODO: Start Climb Button left bumper climb mode only

      //TODO: panic button for b driver left bumper out of climb mode ^^^


      //TODO JoystickButton AutoClimb

  /*
  SmartDashBoard Buttons
  */
      SmartDashboard.putData("0 Wheels", new SetSwerveAngleSafe(mDrivetrain, 0, 0, 0, 0));

      SmartDashboard.putData("Up 1 Angle", new NewHoodTarget(mShooterHood, Math.floor(mShooterHood.getHoodTarget() + 1)));
      SmartDashboard.putData("Down 1 Angle", new NewHoodTarget(mShooterHood, Math.floor(mShooterHood.getHoodTarget() + -1)));

      SmartDashboard.putData("Deploy Intake", new ChangeIntakeState(mIntakeDeploy, true));
      SmartDashboard.putData("Retract Intake", new ChangeIntakeState(mIntakeDeploy, false));
      SmartDashboard.putData("Reset Intake Encoder", new ResetIntakeDeployEncoder(mIntakeDeploy));

    
    /*
    SmartDashboard.putData("0 Hood", new MoveHoodToAngle(mShooterHood, 0.0));
    SmartDashboard.putData("Top Hood", new MoveHoodToAngle(mShooterHood, 140.0));
    SmartDashboard.putData("Bottom Hood", new MoveHoodToAngle(mShooterHood, -140.0));
    // SmartDashboard.putData("100 Hood", new MoveHoodToAngle(mShooterHood,
    // -140.0));
    // SmartDashboard.putData("120 Hood", new MoveHoodToAngle(mShooterHood,
    // -140.0));
    // SmartDashboard.putData("130 Hood", new MoveHoodToAngle(mShooterHood,
    // -140.0));

    SmartDashboard.putData("turret 180", new MoveTurretToAngle(mTurret, 180, 1));
    SmartDashboard.putData("turret 270", new MoveTurretToAngle(mTurret, 270, 1));
    SmartDashboard.putData("turret 90", new MoveTurretToAngle(mTurret, 90, 1));

    
*/
    // SmartDashboard.putData("turret 180", new MoveTurretToAngle(mTurret, 180));
    // SmartDashboard.putData("turret 270", new MoveTurretToAngle(mTurret, 270));
    // SmartDashboard.putData("turret 90", new MoveTurretToAngle(mTurret, 90));
  }

  private void setupAutoSelector() {
    // Auto Commands

    //Command driveStraightNoShootAuto = new AutoFollowPath(mDrivetrain, new StraightPath(2.0, 0).generateSwerveTrajectory(), true, false, 0.0);
    //Command testPathAuto = new AutoFollowPath(mDrivetrain, new TestPath(mDrivetrain, 90.0).generateSwerveTrajectory(), true, true, 90.0);
    //Command p3s1mAuto = new AutoP3S1M(mShooterHood, mShooter, mTurret, cargoBallInterpolator, mCargoFunnel, mTopLift, mBottomLift,
    //  mDrivetrain, mIntake, mIntakeDeploy);
    Command threeBallAuto = new ThreeBallAuto(mShooterHood, mShooter, mTurret, cargoBallInterpolator, mCargoFunnel, mTopLift, mBottomLift, mDrivetrain, mIntake, mIntakeDeploy);
    Command twoBallAuto = new TwoBallAuto(mShooterHood, mShooter, mTurret, cargoBallInterpolator, mCargoFunnel, mTopLift, mBottomLift, mDrivetrain, mIntake, mIntakeDeploy);
    Command fiveBallAuto = new FiveBallAuto(mShooterHood, mShooter, mTurret, cargoBallInterpolator, mCargoFunnel, mTopLift, mBottomLift, mDrivetrain, mIntake, mIntakeDeploy);
    autoChooser = new SendableChooser<>();


    autoChooser.setDefaultOption("Do Nothing", null);
    //autoChooser.addOption("Drive Straight (No Shoot)", driveStraightNoShootAuto);
    //autoChooser.addOption("Test Path", testPathAuto);
    //autoChooser.addOption("P3 Shoot1 Move", p3s1mAuto);
    autoChooser.addOption("3 Ball Auto", threeBallAuto);
    autoChooser.addOption("5 Ball Auto", fiveBallAuto);
    autoChooser.addOption("2 Ball Auto Straight", twoBallAuto);




    SmartDashboard.putData("Auto Selector", autoChooser);
  }

  
  public Command getAutoCommand() {
    return autoChooser.getSelected();
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

  private void initCamera() {
    intakeCamera = CameraServer.startAutomaticCapture();
    intakeCamera.setConnectionStrategy(ConnectionStrategy.kAutoManage);
    intakeCamera.setFPS(20);
    intakeCamera.setResolution(160, 90);

    //virtualCamera = CameraServer.addSwitchedCamera("Drive Camera");
    //virtualCamera.setSource(intakeCamera);
  }

  public void initInterpolator() {
    cargoBallInterpolator = new CargoBallInterpolator();

    // Example adding point
    // cargoBallInterpolator.addDataPoint(new CargoBallDataPoint(distance, Main,
    // Backspin, Hood));
    // cargoBallInterpolator.addDataPoint(new CargoBallDataPoint(35.5, 1750, 2650, -152));
    // cargoBallInterpolator.addDataPoint(new CargoBallDataPoint(48, 1800, 2750, -125));
    // cargoBallInterpolator.addDataPoint(new CargoBallDataPoint(69, 1900, 2900, -98));
    // cargoBallInterpolator.addDataPoint(new CargoBallDataPoint(90.5, 2050, 3150, -36));
    // cargoBallInterpolator.addDataPoint(new CargoBallDataPoint(113.5, 2300, 3150, 31));
    // cargoBallInterpolator.addDataPoint(new CargoBallDataPoint(136.8, 2300, 3500, 89.4));
    // cargoBallInterpolator.addDataPoint(new CargoBallDataPoint(160, 2400, 3600, 130));
    // cargoBallInterpolator.addDataPoint(new CargoBallDataPoint(205, 2750, 3700, 146));
    // cargoBallInterpolator.addDataPoint(new CargoBallDataPoint(255, 3100, 4250, 141));
    // cargoBallInterpolator.addDataPoint(new CargoBallDataPoint(283, 3350, 4600, 136.7));
    cargoBallInterpolator.addDataPoint(new CargoBallDataPoint(37.7, 1750, 2650, -152));
    cargoBallInterpolator.addDataPoint(new CargoBallDataPoint(47.6, 1700, 2550, -125));
    cargoBallInterpolator.addDataPoint(new CargoBallDataPoint(71.6, 1900, 2800, -78.5));
    cargoBallInterpolator.addDataPoint(new CargoBallDataPoint(95.7, 1800, 3050, 2));
    cargoBallInterpolator.addDataPoint(new CargoBallDataPoint(119.4, 2100, 2750, 102));
    cargoBallInterpolator.addDataPoint(new CargoBallDataPoint(141.5, 2250, 2800, 112.6));
    cargoBallInterpolator.addDataPoint(new CargoBallDataPoint(165, 2400, 3000, 148.5));
    cargoBallInterpolator.addDataPoint(new CargoBallDataPoint(188.9, 2600, 3400, 148.5));
    cargoBallInterpolator.addDataPoint(new CargoBallDataPoint(213, 2900, 3550, 148.5));
    cargoBallInterpolator.addDataPoint(new CargoBallDataPoint(236, 3200, 3450, 148.5));
    cargoBallInterpolator.addDataPoint(new CargoBallDataPoint(265, 3300, 3850, 148.5));
    cargoBallInterpolator.addDataPoint(new CargoBallDataPoint(280, 3450, 4700, 138));
  }

}
