// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DeployIntake;
import frc.robot.commands.SpinCargoFunnel;
import frc.robot.commands.SpinIntake;
import frc.robot.commands.StopCargoFunnel;
import frc.robot.commands.StopIntake;
import frc.robot.subsystems.CargoFunnel;
import frc.robot.subsystems.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class StopAutoIntake extends SequentialCommandGroup {

  /** Creates a new AutoIntake. */
  public StopAutoIntake(Intake mIntake, CargoFunnel mCargoFunnel) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(// new DeployIntake(mIntake, false),
        new ParallelCommandGroup(new StopIntake(mIntake), new StopCargoFunnel(mCargoFunnel)));
  }
}