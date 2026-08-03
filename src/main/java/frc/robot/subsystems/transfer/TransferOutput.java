package frc.robot.subsystems.transfer;

import frc.robot.Logger;

import frc.entech.subsystems.SparkOutput;
import frc.entech.subsystems.SubsystemOutput;

public class TransferOutput extends SubsystemOutput {
    private boolean braking = false;
    private SparkOutput transferMotorOutput;

    @Override
    protected void toLog() {
        Logger.recordOutput("TransferOutput/braking", braking);

        if (transferMotorOutput != null) {
            transferMotorOutput.log("TransferOutput/transferMotor");
        }
    }

    /**
     * @return SparkOutput return the transferMotorOutput
     */
    public SparkOutput getTransferMotorOutput() {
        return transferMotorOutput;
    }

    /**
     * @param transferMotorOutput the transferMotorOutput to set
     */
    public void setTransferMotorOutput(SparkOutput transferMotorOutput) {
        this.transferMotorOutput = transferMotorOutput;
    }

    /**
     * @return boolean return the braking
     */
    public boolean isBraking() {
        return braking;
    }

    /**
     * @param braking the braking to set
     */
    public void setBraking(boolean braking) {
        this.braking = braking;
    }

}