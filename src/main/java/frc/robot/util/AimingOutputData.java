package frc.robot.util;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public final class AimingOutputData {
    private final Angle turretAngle;
    private final Angle hoodAngle;
    private final AngularVelocity shooterSpeed;

    public AimingOutputData(Angle turretAngle, Angle hoodAngle, AngularVelocity shooterSpeed) {
        this.turretAngle = turretAngle;
        this.hoodAngle = hoodAngle;
        this.shooterSpeed = shooterSpeed;
    }

    public Angle getTurretAngle() {
        return turretAngle;
    }

    public Angle getHoodAngle() {
        return hoodAngle;
    }

    public AngularVelocity getShooterSpeed() {
        return shooterSpeed;
    }
}
