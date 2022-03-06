// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.DeployIntake;
import frc.robot.commands.RetractIntake;
import frc.robot.commands.StopBottomLift;
import frc.robot.commands.StopCargoFunnel;
import frc.robot.commands.StopIntake;
import frc.robot.commands.StopTopLift;
import frc.robot.subsystems.BottomLift;
import frc.robot.subsystems.CargoFunnel;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.TopLift;
import frc.robot.subsystems.IntakeDeploy;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class StopAutoIntake extends ParallelCommandGroup {

  /** Creates a new AutoIntake. */
  public StopAutoIntake(Intake mIntake, CargoFunnel mCargoFunnel, BottomLift mBottomLift, TopLift mTopLift, IntakeDeploy mIntakeDeploy, boolean mIsPanic) {
    addCommands(
      new RetractIntake(mIntakeDeploy, -2, mIsPanic),
      new StopIntake(mIntake), 
      new StopCargoFunnel(mCargoFunnel), 
      new StopBottomLift(mBottomLift),
      new StopTopLift(mTopLift)
    );
  }
}
