
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.vision.LimeLight.LedMode;

import frc.robot.subsystems.Turret;

public class AutoTurretAim extends CommandBase {

    private Turret mTurret;

    private double turretSetAngle = Turret.getTurretAngle();

    public AutoTurretAim(Turret subsystem) {
        addRequirements(subsystem);

        mTurret = subsystem;
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {

        mTurret.limeLightCamera.setLedMode(LedMode.On);

            if (mTurret.limeLightCamera.hasTarget()) {
                double xOffset = mTurret.limeLightCamera.getTargetXOffset();

                if (Math.abs(xOffset) > 0.5) {
                    turretSetAngle = Turret.getTurretAngle() + xOffset;
                }
                mTurret.goToAngle(turretSetAngle);
            }

            else{
                mTurret.setTurretSpeed(0);
            }
    }

    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
        mTurret.stopTurret();

        mTurret.limeLightCamera.setLedMode(LedMode.Off);
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        return false;
    }
}