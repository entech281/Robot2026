package frc.entech.util.stall;

import com.revrobotics.spark.SparkMax;

/**
 * Adapter that reads the required telemetry from a REV SparkMax.
 * This class is intentionally small and focused only on telemetry extraction.
 */
public final class SparkMaxStallTelemetry implements MotorStallTelemetry {

  private final SparkMax max;

  public SparkMaxStallTelemetry(SparkMax max) {
    this.max = max;
  }

  @Override
  public double appliedOutput() {
    return max.getAppliedOutput();
  }

  @Override
  public double velocity() {
    return max.getEncoder().getVelocity();
  }

  @Override
  public double outputCurrent() {
    return max.getOutputCurrent();
  }
}
