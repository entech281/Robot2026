package frc.robot.subsystems.turret;
import org.littletonrobotics.junction.Logger;

import frc.entech.subsystems.SparkOutput;
import frc.entech.subsystems.SubsystemOutput;

public class TurretOutput extends SubsystemOutput {
  private boolean moving = false;
  private boolean isAtRequestedPosition = false;
  private double requestedPosition = 0.0;
  //TODO: match with physical appearance
  private boolean isAtForwardLimitStall = false;
  private boolean isAtReverseLimitStall = false;
  private double currentPosition = 0.0;
  private boolean isStalled = false;
  private boolean isPastSofterLowerLimit = false;
  private boolean isPastSofterUpperLimit = false;

  private SparkOutput turretMotor;
      
        @Override
        public void toLog() {
          Logger.recordOutput("TurretOutput/moving", moving);
          Logger.recordOutput("TurretOutput/requestedPosition", requestedPosition);
          Logger.recordOutput("TurretOutput/currentPosition", currentPosition);
          Logger.recordOutput("TurretOutput/isAtForwardLimitStall", isAtForwardLimitStall);
          Logger.recordOutput("TurretOutput/isAtReverseLimitStall", isAtReverseLimitStall);
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
      
        public boolean isAtForwardLimitStall() {
          return this.isAtForwardLimitStall;
        }
      
        public void setAtForwardLimitStall(boolean isAtForwardLimitStall) {
          this.isAtForwardLimitStall = isAtForwardLimitStall;
        }
      
        public boolean isAtReverseLimitStall() {
          return this.isAtReverseLimitStall;
        }
      
        public void setAtReverseLimitStall(boolean isAtReverseLimitStall) {
          this.isAtReverseLimitStall = isAtReverseLimitStall;
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

        public boolean isPastSofterLowerLimit() {
          return isPastSofterLowerLimit;
        }

        public void setPastSofterLowerLimit(boolean isPastSofterLowerLimit) {
          this.isPastSofterLowerLimit = isPastSofterLowerLimit;
        }

        public boolean isPastSofterUpperLimit() {
          return isPastSofterUpperLimit;
        }

        public void setPastSofterUpperLimit(boolean isPastSofterUpperLimit) {
          this.isPastSofterUpperLimit = isPastSofterUpperLimit;
        }
}
