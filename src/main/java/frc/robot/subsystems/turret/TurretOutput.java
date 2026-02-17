package frc.robot.subsystems.turret;
import org.littletonrobotics.junction.Logger;

import frc.entech.subsystems.SparkOutput;
import frc.entech.subsystems.SubsystemOutput;

public class TurretOutput extends SubsystemOutput {
  private boolean moving = false;
  private boolean isAtRequestedPosition = false;
  private double requestedPosition = 0.0;
  private boolean isAtForwardLimitStall = false;
  private boolean isAtReverseLimitStall = false;
  private double currentPosition = 0.0;
  private boolean isStalled = false;

  private SparkOutput turretMotor;
      
        @Override
        public void toLog() {
          Logger.recordOutput("TurretOutput/moving", moving);
          Logger.recordOutput("TurretOutput/requestedPosition", requestedPosition);
          Logger.recordOutput("TurretOutput/currentPosition", currentPosition);
          Logger.recordOutput("TurretOutput/isAtForwardLimit", isAtForwardLimit);
          Logger.recordOutput("TurretOutput/isAtReverseLimit", isAtReverseLimit);
          Logger.recordOutput("TurretOutput/isAtRequestedPosition", isAtRequestedPosition);
      
          turretMotor.log("TurretOutput/turretMotor");
        }
      
        public boolean isMoving() {
          return this.moving;
        }
      
        public void setMoving(boolean moving) {
          this.moving = moving;
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
      
        public boolean isAtForwardLimit() {
          return this.isAtForwardLimit;
        }
      
        public void setAtForwardLimit(boolean isAtForwardLimit) {
          this.isAtForwardLimit = isAtForwardLimit;
        }
      
        public boolean isAtReverseLimit() {
          return this.isAtReverseLimit;
        }
      
        public void setAtReverseLimit(boolean isAtReverseLimit) {
          this.isAtReverseLimit = isAtReverseLimit;
        }
      
        public double getRequestedPosition() {
          return this.requestedPosition;
        }
      
        public void setRequestedPosition(double requestedPosition) {
          this.requestedPosition = requestedPosition;
        }
      
        public SparkOutput getTurretMotor() {
          return this.turretMotor;
        }
      
        public void setTurretMotor(SparkOutput turretMotor) {
          this.turretMotor = turretMotor;
        }

        public boolean isStalled() {
          return this.isStalled;
        }

        public void setIsStalled(boolean isStalled) {
          this.isStalled = isStalled;
        }
}
