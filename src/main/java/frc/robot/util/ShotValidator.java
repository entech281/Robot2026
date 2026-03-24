package frc.robot.util;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public class ShotValidator {

    /**
     * Checks if the robot is ready to shoot based on subsystems being at their targets.
     * 
     * @param turretAtTarget Whether the turret is pointing at the target within tolerance.
     * @param hoodAtTarget Whether the hood is at the correct angle within tolerance.
     * @param shooterAtSpeed Whether the shooter wheels are at the target RPM within tolerance.
     * @return True if all conditions are met, allowing a shot to be taken.
     */
    public boolean isReadyToShoot(boolean turretAtTarget, boolean hoodAtTarget, boolean shooterAtSpeed) {
        return turretAtTarget && hoodAtTarget && shooterAtSpeed;
    }

    /**
     * A more detailed check if the turret is within a specific tolerance of the target angle.
     */
    public boolean isTurretAtTarget(Angle currentAngle, Angle targetAngle, Angle tolerance) {
        return Math.abs(currentAngle.minus(targetAngle).in(targetAngle.unit())) <= tolerance.in(targetAngle.unit());
    }

    /**
     * A more detailed check if the shooter is within a specific tolerance of the target speed.
     */
    public boolean isShooterAtSpeed(AngularVelocity currentSpeed, AngularVelocity targetSpeed, AngularVelocity tolerance) {
        return Math.abs(currentSpeed.minus(targetSpeed).in(targetSpeed.unit())) <= tolerance.in(targetSpeed.unit());
    }
}
