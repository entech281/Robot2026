package frc.robot.sensors.HomeTurretSwitch;

import org.littletonrobotics.junction.Logger;

import frc.entech.sensors.SensorOutput;

public class HomeTurretOutput extends SensorOutput {
    private boolean pressed = false;

    @Override
    protected void toLog() {
        Logger.recordOutput("HomeTurretSwitch/pressed", pressed);
    }

    /**
     * @return boolean return the magnetDetected
     */
    public boolean isPressed() {
        return pressed;
    }

    /**
     * @param magnetDetected the magnetDetected to set
     */
    public void setPressed(boolean pressed) {
        this.pressed = pressed;
    }
}
