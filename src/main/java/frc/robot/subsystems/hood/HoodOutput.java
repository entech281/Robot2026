package frc.robot.subsystems.hood;

import org.littletonrobotics.junction.Logger;

import frc.entech.subsystems.SubsystemOutput;

public class HoodOutput extends SubsystemOutput {
    private boolean moving = false;
    private boolean leftBrakeModeEnabled = false;
    private boolean rightBrakeModeEnabled = false;
    private boolean isAtRequestedPosition = false;
    private double requestedPosition = 0.0;
    private boolean isAtUpperLimit = false;
    private boolean isAtLowerLimit = false;
    private double currentPosition = 0.0;

    @Override
    protected void toLog() {
        Logger.recordOutput("HoodOutput/moving", moving);
        Logger.recordOutput("HoodOutput/leftBrakeModeEnabled", leftBrakeModeEnabled);
        Logger.recordOutput("HoodOutput/rightBrakeModeEnabled", rightBrakeModeEnabled);
        Logger.recordOutput("HoodOutput/requestedPosition", requestedPosition);
        Logger.recordOutput("HoodOutput/currentPosition", currentPosition);
        Logger.recordOutput("HoodOutput/isAtUpperLimit", isAtUpperLimit);
        Logger.recordOutput("HoodOutput/isAtLowerLimit", isAtLowerLimit);
        Logger.recordOutput("HoodOutput/isAtRequestedPosition", isAtRequestedPosition);
    }

    public boolean isMoving() {
      return this.moving;
    }

    public void setMoving(boolean moving) {
      this.moving = moving;
    }

    public boolean isLeftBrakeModeEnabled() {
      return this.leftBrakeModeEnabled;
    }

    public void setLeftBrakeModeEnabled(boolean leftBrakeModeEnabled) {
      this.leftBrakeModeEnabled = leftBrakeModeEnabled;
    }

    public boolean isRightBrakeModeEnabled() {
      return this.rightBrakeModeEnabled;
    }

    public void setRightBrakeModeEnabled(boolean rightBrakeModeEnabled) {
      this.rightBrakeModeEnabled = rightBrakeModeEnabled;
    }

    public boolean isAtRequestedPosition() {
      return this.isAtRequestedPosition;
    }

    public void setAtRequestedPosition(boolean isAtRequestedPosition) {
      this.isAtRequestedPosition = isAtRequestedPosition;
    }

    public double getCurrentPosition() {
      return this.currentPosition;
    }  

    public void setCurrentPosition(double currentPosition) {
      this.currentPosition = currentPosition;
    }
  
    public boolean isAtUpperLimit() {
      return this.isAtUpperLimit;
    }

    public void setAtUpperLimit(boolean isAtUpperLimit) {
      this.isAtUpperLimit = isAtUpperLimit;
    }

    public boolean isAtLowerLimit() {
      return this.isAtLowerLimit;
    }

    public void setAtLowerLimit(boolean isAtLowerLimit) {
      this.isAtLowerLimit = isAtLowerLimit;
    }

    public double getRequestedPosition() {
      return this.requestedPosition;
    }

    public void setRequestedPosition(double requestedPosition) {
      this.requestedPosition = requestedPosition;
    }
    
}
