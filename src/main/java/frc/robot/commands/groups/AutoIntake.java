// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.commands.DeployIntake;
import frc.robot.commands.SpinBottomLiftSensor;
import frc.robot.commands.SpinCargoFunnel;
import frc.robot.commands.SpinCargoFunnelSensor;
import frc.robot.commands.SpinIntake;
import frc.robot.commands.SpinTopLift;
import frc.robot.subsystems.BottomLift;
import frc.robot.subsystems.CargoFunnel;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.TopLift;
import frc.robot.subsystems.IntakeDeploy;

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
      new SpinIntake(mIntake, .5),
      new SpinCargoFunnelSensor(mCargoFunnel, mBottomLift, .7, .5),
      new SpinBottomLiftSensor(mBottomLift, .5),
      new SpinTopLift(mTopLift, -.1)
  public AutoIntake(Intake mIntake, CargoFunnel mCargoFunnel, BottomLift mBottomLift, IntakeDeploy mIntakeDeploy) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      // new DeployIntake(mIntake, true),
      new DeployIntake(mIntakeDeploy, 2, false),
      new SpinIntake(mIntake, mIntakeDeploy, .5, false),
      new SpinCargoFunnelSensor(mCargoFunnel, mBottomLift, mIntakeDeploy, .7, .5, false),
      new SpinBottomLiftSensor(mBottomLift, mIntakeDeploy, .5, false)
    );
  }
}
