package frc.robot.utils;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import frc.robot.util.ShotValidator;

public class ShotValidatorTest {

    @Test
    public void testIsReadyToShoot() {
        ShotValidator validator = new ShotValidator();

        assertTrue(validator.isReadyToShoot(true, true, true), "Should be ready when all are true");
        assertFalse(validator.isReadyToShoot(false, true, true), "Should not be ready when turret is false");
        assertFalse(validator.isReadyToShoot(true, false, true), "Should not be ready when hood is false");
        assertFalse(validator.isReadyToShoot(true, true, false), "Should not be ready when shooter is false");
        assertFalse(validator.isReadyToShoot(false, false, false), "Should not be ready when all are false");
    }

    @Test
    public void testIsTurretAtTarget() {
        ShotValidator validator = new ShotValidator();

        assertTrue(validator.isTurretAtTarget(Degrees.of(10), Degrees.of(10.5), Degrees.of(1)), "Within tolerance");
        assertFalse(validator.isTurretAtTarget(Degrees.of(10), Degrees.of(12), Degrees.of(1)), "Outside tolerance");
    }

    @Test
    public void testIsShooterAtSpeed() {
        ShotValidator validator = new ShotValidator();

        assertTrue(validator.isShooterAtSpeed(RPM.of(3000), RPM.of(3050), RPM.of(100)), "Within tolerance");
        assertFalse(validator.isShooterAtSpeed(RPM.of(3000), RPM.of(3200), RPM.of(100)), "Outside tolerance");
    }
}
