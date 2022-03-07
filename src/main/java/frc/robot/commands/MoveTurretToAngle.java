
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.Turret;

public class MoveTurretToAngle extends CommandBase {

    private double mAngle = 0;

      private Turret mTurret;

    public MoveTurretToAngle(Turret subsystem, double angle) {
        mTurret = subsystem;
        mAngle = angle;

        addRequirements(subsystem);
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
            }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
        mTurret.goToAngle(mAngle);
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        return mTurret.onTarget();
    }

    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
        mTurret.stopTurret();
    }
}