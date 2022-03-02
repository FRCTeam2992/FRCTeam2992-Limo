
package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.Turret;

public class MoveTurretToAngle extends CommandBase {

    private double mAngle = 0;

    private double mTimeout = 0;
    private Timer timeoutTimer;

    private Turret mTurret;

    public MoveTurretToAngle(Turret subsystem, double angle, double timeout) {
        mTurret = subsystem;

        mAngle = angle;

        mTimeout = timeout;
        timeoutTimer = new Timer();

        addRequirements(subsystem);
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
        timeoutTimer.reset();
        timeoutTimer.start();
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
        mTurret.goToAngle(mAngle);
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        // return mTurret.turretRotate.atSetpoint() || timeoutTimer.get() >= mTimeout;
        return false;
    }

    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
        mTurret.stopTurret();
    }
}