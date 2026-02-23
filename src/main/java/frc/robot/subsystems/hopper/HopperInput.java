package frc.robot.subsystems.hopper;

import org.littletonrobotics.junction.LogTable;

import frc.entech.subsystems.SubsystemInput;

public class HopperInput implements SubsystemInput {
    private double speed = 0;

    @Override
    public void fromLog(LogTable table) {
        speed = table.get("speed", speed);
    }

    @Override
    public void toLog(LogTable table) {
        table.put("speed", speed);
    }

    public double getSpeed() {
        return speed;
    }

    public void setSpeed(double speed) {
        this.speed = speed;
    }
}