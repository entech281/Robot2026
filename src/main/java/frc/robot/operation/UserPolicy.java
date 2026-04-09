package frc.robot.operation;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Distance;
import frc.robot.livetuning.LiveTuningHandler;
import frc.robot.util.AimingCalculator.VirtualPoseMode;
import edu.wpi.first.units.measure.AngularVelocity;

public class UserPolicy {
  private static final UserPolicy instance = new UserPolicy();

  private boolean twistable = false;
  private boolean aligningToAngle = false;
  private double targetAngle = 0.0;
  private boolean isAutoWon = false;
  private Distance manualShotDistance = Meters.of(4);
  private AngularVelocity shooterRPM = RPM.of(LiveTuningHandler.getInstance().getValue("ShooterSubsystem/SetSpeed"));
  private boolean useBeta = false;
  private double shooterCalculatorSpeedMultiplier = 1.8;
  private boolean useVirtualRotationCompensation = false;
  private VirtualPoseMode virtualPoseMode = VirtualPoseMode.ITERATIVE;

  private UserPolicy() {
    Logger.recordOutput("UserPolicy/twistable", twistable);
    Logger.recordOutput("UserPolicy/aligningToAngle", aligningToAngle);
    Logger.recordOutput("UserPolicy/targetAngle", targetAngle);
    Logger.recordOutput("UserPolicy/isAutoWon", isAutoWon);
    Logger.recordOutput("UserPolicy/hubOffset", manualShotDistance.in(Meters));
    Logger.recordOutput("UserPolicy/shooterRPM", shooterRPM.in(RPM));
    Logger.recordOutput("UserPolicy/useBeta", useBeta);
    Logger.recordOutput("UserPolicy/shooterCalculatorSpeedMultiplier", shooterCalculatorSpeedMultiplier);
  }

  public static UserPolicy getInstance() {
    return instance;
  }

  public boolean isTwistable() {
    return this.twistable;
  }

  public void setIsTwistable(boolean twistable) {
    this.twistable = twistable;
    Logger.recordOutput("UserPolicy/twistable", twistable);
  }

  public boolean isAligningToAngle() {
    return this.aligningToAngle;
  }

  public void setAligningToAngle(boolean aligningToAngle) {
    this.aligningToAngle = aligningToAngle;
    Logger.recordOutput("UserPolicy/aligningToAngle", aligningToAngle);
  }

  public double getTargetAngle() {
    return this.targetAngle;
  }

  public void setTargetAngle(double targetAngle) {
    this.targetAngle = targetAngle;
    Logger.recordOutput("UserPolicy/targetAngle", targetAngle);
  }

  public boolean isAutoWon() {
    return this.isAutoWon;
  }

  public void setIsAutoWon(boolean isAutoWon) {
    this.isAutoWon = isAutoWon;
    Logger.recordOutput("UserPolicy/isAutoWon", isAutoWon);
  }

  public Distance getHubOffset() {
    return manualShotDistance;
  }

  public void setHubOffset(Distance manualShotDistance) {
    Logger.recordOutput("UserPolicy/hubOffset", manualShotDistance.in(Meters));
    this.manualShotDistance = manualShotDistance;
  }

  public void setShooterRPM(AngularVelocity shooterRPM) {
    Logger.recordOutput("UserPolicy/shooterRPM", shooterRPM.in(RPM));
    this.shooterRPM = shooterRPM;
  }

  public AngularVelocity getShooterRPM() {
    return shooterRPM;
  }

  public void setUseBeta(boolean useBeta) {
    this.useBeta = useBeta;
    Logger.recordOutput("UserPolicy/useBeta", useBeta);
  }

  public void setShooterCalculatorSpeedMultiplier(double shooterCalculatorSpeedMultiplier) {
    this.shooterCalculatorSpeedMultiplier = shooterCalculatorSpeedMultiplier;
    Logger.recordOutput("UserPolicy/shooterCalculatorSpeedMultiplier", shooterCalculatorSpeedMultiplier);
  }

  public double getShooterCalculatorSpeedMultiplier() {
    return shooterCalculatorSpeedMultiplier;
  }

  public boolean getUseBeta() {
    return useBeta;
  }

    /**
     * @return boolean return the useVirtualRotationCompensation
     */
    public boolean isUseVirtualRotationCompensation() {
        return useVirtualRotationCompensation;
    }

    /**
     * @param useVirtualRotationCompensation the useVirtualRotationCompensation to set
     */
    public void setUseVirtualRotationCompensation(boolean useVirtualRotationCompensation) {
        this.useVirtualRotationCompensation = useVirtualRotationCompensation;
    }

    /**
     * @return VirtualPoseMode return the virtualPoseMode
     */
    public VirtualPoseMode getVirtualPoseMode() {
        return virtualPoseMode;
    }

    /**
     * @param virtualPoseMode the virtualPoseMode to set
     */
    public void setVirtualPoseMode(VirtualPoseMode virtualPoseMode) {
        this.virtualPoseMode = virtualPoseMode;
    }

}
