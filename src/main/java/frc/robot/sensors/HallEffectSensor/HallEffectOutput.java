package frc.robot.sensors.HallEffectSensor;

import org.littletonrobotics.junction.Logger;

import frc.entech.sensors.SensorOutput;

public class HallEffectOutput extends SensorOutput {
    private double value = 0;

    @Override
    protected void toLog() {
        Logger.recordOutput("HallEffectSensor/value", value);
    }

    /**
     * @return double return the value
     */
    public double getValue() {
        return value;
    }

    /**
     * @param value the value to set
     */
    public void setValue(double value) {
        this.value = value;
    }

}
