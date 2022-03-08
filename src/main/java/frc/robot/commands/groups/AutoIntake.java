// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.SetBottomLiftCommanded;
import frc.robot.commands.SetCargoFunnelCommanded;
import frc.robot.commands.SetIntakeCommanded;
import frc.robot.commands.SetTopLiftCommanded;
import frc.robot.subsystems.BottomLift;
import frc.robot.subsystems.CargoFunnel;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.TopLift;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoIntake extends ParallelCommandGroup {

  /** Creates a new AutoIntake. */
  public AutoIntake(Intake mIntake, CargoFunnel mCargoFunnel, BottomLift mBottomLift, TopLift mTopLift) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      // TODO: new DeployIntake(idk what goes here);
      new SetIntakeCommanded(mIntake, true, 0.5),
      new SetCargoFunnelCommanded(mCargoFunnel, true, true, .7, .5, 0.0),
      new SetBottomLiftCommanded(mBottomLift, true, true, 0.5, 0.0, 0.11),
      new SetTopLiftCommanded(mTopLift, true, -0.1)
    );
  }
}
