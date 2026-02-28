package frc.robot.subsystems.hopper;

import org.littletonrobotics.junction.Logger;

import frc.entech.subsystems.SparkOutput;
import frc.entech.subsystems.SubsystemOutput;

public class HopperOutput extends SubsystemOutput {
    private SparkOutput hopperMotorOutput;
    private boolean atLowerLimit = false;
    private boolean isStalled = false;

    public boolean isStalled() {
        return isStalled;
    }

    public void setStalled(boolean isStalled) {
        this.isStalled = isStalled;
    }

    @Override
    protected void toLog() {
        if (hopperMotorOutput != null) {
            hopperMotorOutput.log("HopperOutput/hopperMotor");
        }
        Logger.recordOutput("HopperOutput/atLowerLimit", atLowerLimit);
        Logger.recordOutput("HopperOutput/isStalled", isStalled);
    }

    public SparkOutput getHopperMotorOutput() {
        return hopperMotorOutput;
    }

    public void setHopperMotorOutput(SparkOutput hopperMotorOutput) {
        this.hopperMotorOutput = hopperMotorOutput;
    }

    public boolean isAtLowerLimit() {
        return atLowerLimit;
    }

    public void setAtLowerLimit(boolean atLowerLimit) {
        this.atLowerLimit = atLowerLimit;
    }
}