package frc.entech.util.stall;

import com.revrobotics.spark.SparkBase;

/**
 * Adapter that reads the required telemetry from a REV SparkMax.
 * This class is intentionally small and focused only on telemetry extraction.
 */
public final class SparkStallTelemetry implements MotorStallTelemetry {
 
  private final SparkBase spark;

  public SparkStallTelemetry(SparkBase spark) {
    this.spark = spark;
  }

  @Override
  public double thresholdVoltage() {
    return spark.getAppliedOutput();
  }

  @Override
  public double thresholdVelocity() {
    return spark.getEncoder().getVelocity();
  }

  @Override
  public double thresholdCurrent() {
    return spark.getOutputCurrent();
  }
}
