package frc.robot.sensors.HallEffectSensor;

import org.littletonrobotics.junction.Logger;

import frc.entech.sensors.SensorOutput;

public class HallEffectOutput extends SensorOutput {
    private boolean magnetDetected = false;

    @Override
    protected void toLog() {
        Logger.recordOutput("HallEffectSensor/magnetDetected", magnetDetected);
    }

    /**
     * @return boolean return the magnetDetected
     */
    public boolean isMagnetDetected() {
        return magnetDetected;
    }

    /**
     * @param magnetDetected the magnetDetected to set
     */
    public void setMagnetDetected(boolean magnetDetected) {
        this.magnetDetected = magnetDetected;
    }
}
