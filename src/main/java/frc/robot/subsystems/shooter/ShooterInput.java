package frc.robot.subsystems.shooter;

import frc.entech.subsystems.SubsystemInput;
import frc.robot.Logger;

public class ShooterInput extends SubsystemInput {
    private double speed = 0.0;

    @Override
    public void toLog() {
        Logger.recordOutput("ShooterInput/speed", speed);
    }

    /**
     * @return double return the speed
     */
    public double getSpeed() {
        return speed;
    }

    /**
     * @param speed the speed to set
     */
    public void setSpeed(double speed) {
        this.speed = speed;
    }
}
