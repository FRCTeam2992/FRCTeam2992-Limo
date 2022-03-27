// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.lib.Ranging.CargoBallInterpolator;
import frc.robot.commands.AutoFollowPath;
import frc.robot.commands.AutoLimelightHood;
import frc.robot.commands.AutoLimelightMainShooter;
import frc.robot.commands.AutoLimelightSecondShooter;
import frc.robot.commands.AutoTurretAimAutonomous;
import frc.robot.commands.ChangeIntakeState;
import frc.robot.commands.NewHoodTarget;
import frc.robot.commands.ResetGyro;
import frc.robot.commands.SetShooterCommanded;
import frc.robot.commands.SetShooterSpeedTargets;
import frc.robot.commands.SetTurretTargetAngle;
import frc.robot.commands.StartHood;
import frc.robot.commands.groups.AutoIntake;
import frc.robot.commands.groups.AutoShootAutonomous;
import frc.robot.paths.FiveBallFinalPath;
import frc.robot.paths.ThreeBallForFivePath;
import frc.robot.paths.ThreeBallMainPath;
import frc.robot.subsystems.BottomLift;
import frc.robot.subsystems.CargoFunnel;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakeDeploy;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterHood;
import frc.robot.subsystems.TopLift;
import frc.robot.subsystems.Turret;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FiveBallAuto extends ParallelCommandGroup {

  /** Creates a new AutoIntake. */
  public FiveBallAuto(ShooterHood mShooterHood, Shooter mShooter, Turret mTurret, 
      CargoBallInterpolator mInterpolator, CargoFunnel mCargoFunnel, TopLift mTopLift,
      BottomLift mBottomLift, Drivetrain mDrivetrain, Intake mIntake, IntakeDeploy mIntakeDeploy) {
    addCommands(
      //new NewHoodTarget(mShooterHood, 0.0),
      //new StartHood(mShooterHood),
      //new SetShooterSpeedTargets(mShooter, 1500, 2000),
      
      new ResetGyro(mDrivetrain, 88.5),
      new NewHoodTarget(mShooterHood, -67.5),
      new StartHood(mShooterHood),
      new SetShooterSpeedTargets(mShooter, 1700, 2700),
      new SetShooterCommanded(mShooter, true),
      new AutoTurretAimAutonomous(mTurret, true, 194),
      new AutoLimelightHood(mTurret, mShooterHood, mInterpolator),
      new AutoLimelightMainShooter(mTurret, mShooter, mInterpolator),
      new AutoLimelightSecondShooter(mTurret, mShooter, mInterpolator),
      new SequentialCommandGroup(
        new WaitCommand(0.25),
        // new WaitCommand(0.040),
        new AutoShootAutonomous(mCargoFunnel, mTopLift, mBottomLift, mShooter, mShooterHood, mTurret, mDrivetrain).withTimeout(0.75),
        new ChangeIntakeState(mIntakeDeploy, true),
        new AutoIntake(mIntake, mCargoFunnel, mBottomLift, mTopLift, mIntakeDeploy),
        new NewHoodTarget(mShooterHood, 106),
        new SetTurretTargetAngle(mTurret, true, 98.5),
        new SetShooterSpeedTargets(mShooter, 2266, 2850),
        new AutoFollowPath(mDrivetrain, new ThreeBallForFivePath(mDrivetrain, 88.5).generateSwerveTrajectory(), true, false, 0.0).withTimeout(5),
        new WaitCommand(0.5),
        new AutoShootAutonomous(mCargoFunnel, mTopLift, mBottomLift, mShooter, mShooterHood, mTurret, mDrivetrain).withTimeout(1.3),
        new AutoIntake(mIntake, mCargoFunnel, mBottomLift, mTopLift, mIntakeDeploy),
        new SetTurretTargetAngle(mTurret, true, 174),
        new SetShooterSpeedTargets(mShooter, 2377, 2877),
        new NewHoodTarget(mShooterHood, 117),
        new AutoFollowPath(mDrivetrain, new FiveBallFinalPath(mDrivetrain, -130).generateSwerveTrajectory(), false, false, -130.0).withTimeout(7.5),
        new AutoShootAutonomous(mCargoFunnel, mTopLift, mBottomLift, mShooter, mShooterHood, mTurret, mDrivetrain).withTimeout(3)
      )
    );
  }
}
