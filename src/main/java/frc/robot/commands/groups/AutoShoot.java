// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.commands.SpinBottomLift;
import frc.robot.commands.SpinCargoFunnel;
import frc.robot.commands.SpinTopLift;
import frc.robot.subsystems.BottomLift;
import frc.robot.subsystems.CargoFunnel;
import frc.robot.subsystems.TopLift;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoShoot extends ParallelRaceGroup {

  /** Creates a new AutoShoot. */
  public AutoShoot(CargoFunnel mCargoFunnel, TopLift mTopLift, BottomLift mBottomLift) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SpinCargoFunnel(mCargoFunnel, 1),
      new SpinTopLift(mTopLift, 1),
      new SpinBottomLift(mBottomLift, 1)
    );
  }
}
