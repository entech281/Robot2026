package frc.entech.util.stall;

/**
 * Minimal telemetry needed for stall detection.
 *
 * NOTE: MotorStallDetector assumes ONE detector instance per device.
 */
public interface MotorStallTelemetry {
 
  /** Applied output in the range [-1, 1]. */
  double thresholdVoltage();

  /** Mechanism velocity in user units (typically RPM for Spark encoder). */
  double thresholdVelocity();

  /** Output current in amps. */
  double thresholdCurrent();
}
