package frc.entech.util.stall;

import com.revrobotics.spark.SparkBase;
import java.util.Objects;
 
public final class MotorStallDetector {

  private final double minAppliedOutput;
  private final double minMovingVelocity;
  private final double maxAllowableCurrent;
  private final int requiredLoops;

  private int consecutiveTrueLoops = 0;

  private MotorStallDetector(
      double minAppliedOutput, double maxAbsVelocity, double minCurrentAmps, int requiredLoops) {
    validate(minAppliedOutput, maxAbsVelocity, minCurrentAmps, requiredLoops);

    this.minAppliedOutput = minAppliedOutput;
    this.minMovingVelocity = maxAbsVelocity;
    this.maxAllowableCurrent = minCurrentAmps;
    this.requiredLoops = requiredLoops;
  }

  public static Builder builder() {
    return new Builder();
  }

  public boolean isStalled(SparkBase spark) {
    return isStalled(new SparkStallTelemetry(spark));
  }

  public boolean isStalled(MotorStallTelemetry t) {
    Objects.requireNonNull(t, "telemetry");

    final boolean looksStalled =
        Math.abs(t.thresholdVoltage()) >= minAppliedOutput
            && Math.abs(t.thresholdVelocity()) <= minMovingVelocity
            && t.thresholdCurrent() >= maxAllowableCurrent;

    if (looksStalled) {
      incrementStallCountUpToLimit();
    } else {
      resetStallCount();
    }

    return isStalledEnoughCounts();
  }

  public void reset() {
    resetStallCount();
  }

  public int getConsecutiveTrueLoops() {
    return consecutiveTrueLoops;
  }

  public double getMinAppliedOutput() {
    return minAppliedOutput;
  }

  public double getMinMovingVelocity() {
    return minMovingVelocity;
  }

  public double getMaxAllowableCurrent() {
    return maxAllowableCurrent;
  }

  public int getRequiredLoops() {
    return requiredLoops;
  }

  private void incrementStallCountUpToLimit() {
    if (consecutiveTrueLoops < requiredLoops) {
      consecutiveTrueLoops++;
    }
  }

  private void resetStallCount() {
    consecutiveTrueLoops = 0;
  }

  private boolean isStalledEnoughCounts() {
    return consecutiveTrueLoops >= requiredLoops;
  }

  private static void validate(
      double minAppliedOutput, double maxAbsVelocity, double minCurrentAmps, int requiredLoops) {
    if (minAppliedOutput < 0.0 || minAppliedOutput > 1.0) {
      throw new IllegalArgumentException("minAppliedOutput must be in [0, 1]");
    }
    if (maxAbsVelocity < 0.0) {
      throw new IllegalArgumentException("maxAbsVelocity must be >= 0");
    }
    if (minCurrentAmps < 0.0) {
      throw new IllegalArgumentException("minCurrentAmps must be >= 0");
    }
    if (requiredLoops <= 0) {
      throw new IllegalArgumentException("requiredLoops must be >= 1");
    }
  }

  public static final class Builder {
    private Double minAppliedOutput;
    private Double maxAbsVelocity;
    private Double minCurrentAmps;
    private Integer requiredLoops;

    private Builder() {}

    public Builder minAppliedOutput(double value) {
      this.minAppliedOutput = value;
      return this;
    }

    public Builder maxAbsVelocity(double value) {
      this.maxAbsVelocity = value;
      return this;
    }

    public Builder minCurrentAmps(double value) {
      this.minCurrentAmps = value;
      return this;
    }

    public Builder requiredLoops(int value) {
      this.requiredLoops = value;
      return this;
    }

    public MotorStallDetector build() {
      return new MotorStallDetector(
          requireSet(minAppliedOutput, "minAppliedOutput"),
          requireSet(maxAbsVelocity, "maxAbsVelocity"),
          requireSet(minCurrentAmps, "minCurrentAmps"),
          requireSet(requiredLoops, "requiredLoops"));
    }

    public static MotorStallDetector defaults() {
      return MotorStallDetector.builder()
          .minAppliedOutput(0.2)
          .maxAbsVelocity(1e-3)
          .minCurrentAmps(25.0)
          .requiredLoops(5)
          .build();
    }

    private static double requireSet(Double v, String name) {
      if (v == null) throw new IllegalStateException(name + " was not set");
      return v;
    }

    private static int requireSet(Integer v, String name) {
      if (v == null) throw new IllegalStateException(name + " was not set");
      return v;
    }
  }
}
