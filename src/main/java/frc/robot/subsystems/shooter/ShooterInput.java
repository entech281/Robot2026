package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.LogTable;

import frc.entech.subsystems.SubsystemInput;

public class ShooterInput implements SubsystemInput {
    private double speed = 0.0;

    @Override
    public void toLog(LogTable table) {
        table.put("ShooterOutput/speed", speed);
    }

    @Override
    public void fromLog(LogTable table) {
        speed = table.get("ShooterOutput/speed", 0);
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
