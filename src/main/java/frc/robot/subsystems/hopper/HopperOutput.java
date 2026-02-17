package frc.robot.subsystems.hopper;

import frc.entech.subsystems.SparkOutput;
import frc.entech.subsystems.SubsystemOutput;

public class HopperOutput extends SubsystemOutput {
    private SparkOutput hopperMotorOutput;

    @Override
    protected void toLog() {
        if (hopperMotorOutput != null) {
            hopperMotorOutput.log("HopperOutput/hopperMotor");
        }
    }

    public SparkOutput getHopperMotorOutput() {
        return hopperMotorOutput;
    }

    public void setHopperMotorOutput(SparkOutput hopperMotorOutput) {
        this.hopperMotorOutput = hopperMotorOutput;
    }
}