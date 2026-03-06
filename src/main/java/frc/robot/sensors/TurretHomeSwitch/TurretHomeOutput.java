package frc.robot.sensors.TurretHomeSwitch;

import org.littletonrobotics.junction.Logger;

import frc.entech.sensors.SensorOutput;

public class TurretHomeOutput extends SensorOutput {
    private boolean pressed = false;

    @Override
    protected void toLog() {
        Logger.recordOutput("TurretHomeSwitch/pressed", pressed);
    }

    /**
     * @return boolean return the pressed
     */
    public boolean isPressed() {
        return pressed;
    }

    /**
     * @param pressed the pressed to set
     */
    public void setPressed(boolean pressed) {
        this.pressed = pressed;
    }
}
