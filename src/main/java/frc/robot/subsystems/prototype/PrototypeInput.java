package frc.robot.subsystems.prototype;

import org.littletonrobotics.junction.LogTable;

import frc.entech.subsystems.SubsystemInput;

public class PrototypeInput implements SubsystemInput{
    private double speed;
    private String key = "prototype1Input";
    @Override
    public void toLog(LogTable table) {
        table.put(key + "/speed",speed);
    }

    @Override
    public void fromLog(LogTable table) {
        speed = table.get(key + "/speed", 0.0);
    }
    public double getSpeed(){
        return speed;
    }
    public void setSpeed(double speed){
        this.speed = speed;
    }
}
