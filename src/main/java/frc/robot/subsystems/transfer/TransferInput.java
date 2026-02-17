package frc.robot.subsystems.transfer;

import org.littletonrobotics.junction.LogTable;

import frc.entech.subsystems.SubsystemInput;

public class TransferInput implements SubsystemInput {
    private double speed = 0.0;

    @Override
    public void fromLog(LogTable table) {
        speed = table.get("speed", 0.0);
    }

    @Override
    public void toLog(LogTable table) {
        table.put("speed", speed);
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
