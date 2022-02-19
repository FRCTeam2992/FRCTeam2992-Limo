// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DeployClimb;
import frc.robot.commands.DeployIntake;
import frc.robot.commands.DeployStationaryHook;
import frc.robot.commands.MoveHoodToAngle;
import frc.robot.commands.MoveTurretToAngle;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.ShooterHood;
import frc.robot.subsystems.Turret;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ClimbMode extends SequentialCommandGroup {
  /** Creates a new ClimbMode. */
  public ClimbMode(Climb mClimb, Turret mTurret, Intake mIntake, ShooterHood mShooterHood) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new MoveTurretToAngle(mTurret, 0.0, 0), new MoveHoodToAngle(mShooterHood, 0), 
    new DeployIntake(mIntake, false),new DeployClimb(mClimb, false), new DeployStationaryHook(mClimb, true));
  }
}
