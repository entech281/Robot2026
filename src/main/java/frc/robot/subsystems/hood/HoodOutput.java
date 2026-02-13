package frc.robot.subsystems.hood;

import org.littletonrobotics.junction.Logger;

import frc.entech.subsystems.SparkOutput;
import frc.entech.subsystems.SubsystemOutput;

public class HoodOutput extends SubsystemOutput {

    private boolean moving = false;
    private boolean isAtRequestedPosition = false;
    private double requestedPosition = 0.0;
    private SparkOutput hoodMotor;
    private double currentPosition = 0.0;

    @Override
    protected void toLog() {
        Logger.recordOutput("HoodOutput/moving", moving);
        Logger.recordOutput("HoodOutput/requestedPosition", requestedPosition);
        Logger.recordOutput("HoodOutput/currentPosition", currentPosition);
        Logger.recordOutput("HoodOutput/isAtRequestedPosition", isAtRequestedPosition);
      
        hoodMotor.log("HoodOutput/hoodMotor");
    }

    public boolean isMoving() {
        return moving;
    }

    public void setMoving(boolean moving) {
        this.moving = moving;
    }

    public boolean isAtRequestedPosition() {
        return isAtRequestedPosition;
    }

    public void setAtRequestedPosition(boolean isAtRequestedPosition) {
        this.isAtRequestedPosition = isAtRequestedPosition;
    }

    public double getRequestedPosition() {
        return requestedPosition;
    }

    public void setRequestedPosition(double requestedPosition) {
        this.requestedPosition = requestedPosition;
    }

    public SparkOutput getHoodMotor() {
        return hoodMotor;
    }

    public void setHoodMotor(SparkOutput hoodMotor) {
        this.hoodMotor = hoodMotor;
    }

    public double getCurrentPosition() {
        return currentPosition;
    }

    public void setCurrentPosition(double currentPosition) {
        this.currentPosition = currentPosition;
    }
    
}
