package frc.robot.subsystems.intake;

import frc.entech.subsystems.SubsystemInput;
import frc.robot.Logger;

public class IntakeInput extends SubsystemInput {
    private double speed = 0;

    @Override
    public void toLog() {
        Logger.recordOutput("IntakeInput/speed", speed);
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
