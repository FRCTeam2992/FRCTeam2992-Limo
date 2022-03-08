// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.SpinBottomLiftSensor;
import frc.robot.commands.SpinCargoFunnelSensor;
import frc.robot.commands.SpinTopLift;
import frc.robot.subsystems.BottomLift;
import frc.robot.subsystems.CargoFunnel;
import frc.robot.subsystems.TopLift;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoIntake extends ParallelCommandGroup {

  /** Creates a new AutoIntake. */
  public AutoIntake(CargoFunnel mCargoFunnel, BottomLift mBottomLift, TopLift mTopLift) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      // TODO: new DeployIntake(idk what goes here);
      new SpinCargoFunnelSensor(mCargoFunnel, mBottomLift, .7, .5),
      new SpinBottomLiftSensor(mBottomLift, .5),
      new SpinTopLift(mTopLift, -.1)
    );
  }
}
