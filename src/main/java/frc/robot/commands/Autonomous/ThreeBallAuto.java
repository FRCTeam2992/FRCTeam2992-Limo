// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import com.fasterxml.jackson.databind.ser.std.NumberSerializers.ShortSerializer;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.lib.Ranging.CargoBallInterpolator;
import frc.robot.commands.AutoFollowPath;
import frc.robot.commands.AutoLimelightHood;
import frc.robot.commands.AutoLimelightMainShooter;
import frc.robot.commands.AutoLimelightSecondShooter;
import frc.robot.commands.AutoTurretAim;
import frc.robot.commands.MoveTurretToAngle;
import frc.robot.commands.NewHoodTarget;
import frc.robot.commands.SetShooterCommanded;
import frc.robot.commands.SetShooterSpeedTargets;
import frc.robot.commands.SetTopLiftCommanded;
import frc.robot.commands.StartHood;
import frc.robot.commands.StopTopLift;
import frc.robot.commands.groups.AutoIntake;
import frc.robot.commands.groups.AutoLimelightRange;
import frc.robot.commands.groups.AutoShoot;
import frc.robot.commands.groups.AutoShootAutonomous;
import frc.robot.commands.groups.StopAutoIntake;
import frc.robot.paths.StraightPath;
import frc.robot.paths.ThreeBallPath;
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
public class ThreeBallAuto extends ParallelCommandGroup {

  /** Creates a new AutoIntake. */
  public ThreeBallAuto(ShooterHood mShooterHood, Shooter mShooter, Turret mTurret, 
      CargoBallInterpolator mInterpolator, CargoFunnel mCargoFunnel, TopLift mTopLift,
      BottomLift mBottomLift, Drivetrain mDrivetrain, Intake mIntake, IntakeDeploy mIntakeDeploy) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      // TODO: new DeployIntake(idk what goes here);
      //new NewHoodTarget(mShooterHood, 0.0),
      //new StartHood(mShooterHood),
      //new SetShooterSpeedTargets(mShooter, 1500, 2000),
      new SetShooterCommanded(mShooter, true),
      new AutoTurretAim(mTurret),
      new AutoLimelightHood(mTurret, mShooterHood, mInterpolator),
      new AutoLimelightMainShooter(mTurret, mShooter, mInterpolator),
      new AutoLimelightSecondShooter(mTurret, mShooter, mInterpolator),
      new SequentialCommandGroup(
        // new WaitCommand(0.040),
        new AutoShootAutonomous(mCargoFunnel, mTopLift, mBottomLift, mShooter, mShooterHood, mTurret, mDrivetrain).withTimeout(1.0),
        new AutoIntake(mIntake, mCargoFunnel, mBottomLift, mTopLift, mIntakeDeploy),
        new AutoFollowPath(mDrivetrain, new ThreeBallPath(mDrivetrain, 88.5).generateSwerveTrajectory(), true, true, 88.5).withTimeout(5),
        new AutoShootAutonomous(mCargoFunnel, mTopLift, mBottomLift, mShooter, mShooterHood, mTurret, mDrivetrain).withTimeout(3),
        new StopAutoIntake(mIntake, mCargoFunnel, mBottomLift, mTopLift, mIntakeDeploy),
        new SetShooterCommanded(mShooter, false)
      )
    );
  }
}
