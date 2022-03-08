// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.SetCargoFunnelCommanded;
import frc.robot.commands.SetTopLiftCommanded;
import frc.robot.commands.StopBottomLift;
import frc.robot.subsystems.BottomLift;
import frc.robot.subsystems.CargoFunnel;
import frc.robot.subsystems.TopLift;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class StopAutoIntake extends ParallelCommandGroup {

  /** Creates a new AutoIntake. */
  public StopAutoIntake(CargoFunnel mCargoFunnel, BottomLift mBottomLift, TopLift mTopLift) {
    addCommands(
      // TODO:  Figure out deploy intake new DeployIntake(mIntake, false),
      new SetCargoFunnelCommanded(mCargoFunnel, false, false, 0.0, 0.0, 0.0),
      new StopBottomLift(mBottomLift),
      new SetTopLiftCommanded(mTopLift, false, 0.0)
    );
  }
}
