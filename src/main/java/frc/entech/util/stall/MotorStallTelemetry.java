package frc.entech.util.stall;

/**
 * Minimal telemetry needed for stall detection.
 *
 * NOTE: MotorStallDetector assumes ONE detector instance per device.
 */
public interface MotorStallTelemetry {

  /** Applied output in the range [-1, 1]. */
  double appliedOutput();

  /** Mechanism velocity in user units (typically RPM for Spark encoder). */
  double velocity();

  /** Output current in amps. */
  double outputCurrent();
}
