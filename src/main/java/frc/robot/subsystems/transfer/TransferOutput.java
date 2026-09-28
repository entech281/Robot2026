package frc.robot.subsystems.transfer;

import org.littletonrobotics.junction.Logger;

import frc.entech.subsystems.SparkOutput;
import frc.entech.subsystems.SubsystemOutput;

public class TransferOutput extends SubsystemOutput {
    private boolean braking = false;
    private boolean motor1Connected = false;
    private boolean motor2Connected = false;

    private SparkOutput transferMotor1Output;
    private SparkOutput transferMotor2Output;

    @Override
    protected void toLog() {
        Logger.recordOutput("TransferOutput/braking", braking);
        Logger.recordOutput("TransferOutput/motor1Connected", motor1Connected);
        Logger.recordOutput("TransferOutput/motor2Connected", motor2Connected);

        if (transferMotor1Output != null) {
            transferMotor1Output.log("TransferOutput/transferMotor1");
        }

        if (transferMotor2Output != null) {
            transferMotor2Output.log("TransferOutput/transferMotor2");
        }
    }

    public SparkOutput getTransferMotor1Output() {
        return transferMotor1Output;
    }

    public void setTransferMotor1Output(SparkOutput transferMotor1Output) {
        this.transferMotor1Output = transferMotor1Output;
    }

    public SparkOutput getTransferMotor2Output() {
        return transferMotor2Output;
    }

    public void setTransferMotor2Output(SparkOutput transferMotor2Output) {
        this.transferMotor2Output = transferMotor2Output;
    }

    public boolean isMotor1Connected() {
        return motor1Connected;
    }

    public void setMotor1Connected(boolean motor1Connected) {
        this.motor1Connected = motor1Connected;
    }

    public boolean isMotor2Connected() {
        return motor2Connected;
    }

    public void setMotor2Connected(boolean motor2Connected) {
        this.motor2Connected = motor2Connected;
    }

    public boolean isBraking() {
        return braking;
    }

    public void setBraking(boolean braking) {
        this.braking = braking;
    }
}