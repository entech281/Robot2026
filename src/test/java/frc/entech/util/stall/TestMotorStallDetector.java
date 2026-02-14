package frc.entech.util.stall;

import static org.junit.jupiter.api.Assertions.*;

import java.util.stream.Stream;

import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.MethodSource;

public class TestMotorStallDetector {

  private static final class FakeTelemetry implements MotorStallTelemetry {
    double appliedOutput;
    double velocity;
    double current;

    int callsAppliedOutput = 0;
    int callsVelocity = 0;
    int callsCurrent = 0;

    FakeTelemetry set(double appliedOutput, double velocity, double current) {
      this.appliedOutput = appliedOutput;
      this.velocity = velocity;
      this.current = current;
      return this;
    }

    void resetCallCounts() {
      callsAppliedOutput = 0;
      callsVelocity = 0;
      callsCurrent = 0;
    }

    @Override
    public double appliedOutput() {
      callsAppliedOutput++;
      return appliedOutput;
    }

    @Override
    public double velocity() {
      callsVelocity++;
      return velocity;
    }

    @Override
    public double outputCurrent() {
      callsCurrent++;
      return current;
    }
  }

  private static final double MIN_OUT = 0.20;
  private static final double MAX_VEL = 0.05;
  private static final double MIN_CUR = 10.0;

  private static final double OUT_ABOVE_POS = 0.25;
  private static final double OUT_ABOVE_NEG = -0.25;
  private static final double OUT_BELOW_POS = 0.15;
  private static final double OUT_BELOW_NEG = -0.15;

  private static final double VEL_WITHIN_POS = 0.04;
  private static final double VEL_WITHIN_NEG = -0.04;
  private static final double VEL_OUTSIDE_POS = 0.06;
  private static final double VEL_OUTSIDE_NEG = -0.06;

  private static final double CUR_ABOVE = 12.0;
  private static final double CUR_BELOW = 8.0;

  private static MotorStallDetector makeDetector(int requiredLoops) {
    return MotorStallDetector.builder()
        .minAppliedOutput(MIN_OUT)
        .maxAbsVelocity(MAX_VEL)
        .minCurrentAmps(MIN_CUR)
        .requiredLoops(requiredLoops)
        .build();
  }

  static Stream<Arguments> truthTableCases() {
    return Stream.of(
        Arguments.of(true, true, true, true),
        Arguments.of(true, true, false, false),
        Arguments.of(true, false, true, false),
        Arguments.of(true, false, false, false),
        Arguments.of(false, true, true, false),
        Arguments.of(false, true, false, false),
        Arguments.of(false, false, true, false),
        Arguments.of(false, false, false, false));
  }

  private static double pickOutput(boolean outputOk, boolean negative) {
    if (outputOk) return negative ? OUT_ABOVE_NEG : OUT_ABOVE_POS;
    return negative ? OUT_BELOW_NEG : OUT_BELOW_POS;
  }

  private static double pickVelocity(boolean velocityOk, boolean negative) {
    if (velocityOk) return negative ? VEL_WITHIN_NEG : VEL_WITHIN_POS;
    return negative ? VEL_OUTSIDE_NEG : VEL_OUTSIDE_POS;
  }

  private static double pickCurrent(boolean currentOk) {
    return currentOk ? CUR_ABOVE : CUR_BELOW;
  }

  @ParameterizedTest(name = "outputOk={0}, velocityOk={1}, currentOk={2} => stalled={3}")
  @MethodSource("truthTableCases")
  @DisplayName("Truth table: all 8 permutations (positive values)")
  public void truthTablePositiveValues(
      boolean outputOk, boolean velocityOk, boolean currentOk, boolean expectedStalled) {
    MotorStallDetector det = makeDetector(1);
    FakeTelemetry t =
        new FakeTelemetry()
            .set(
                pickOutput(outputOk, false),
                pickVelocity(velocityOk, false),
                pickCurrent(currentOk));

    assertEquals(expectedStalled, det.isStalled(t));
  }

  @ParameterizedTest(name = "NEG outputOk={0}, velocityOk={1}, currentOk={2} => stalled={3}")
  @MethodSource("truthTableCases")
  @DisplayName("Truth table: all 8 permutations (negative output & velocity)")
  public void truthTableNegativeOutputAndVelocity(
      boolean outputOk, boolean velocityOk, boolean currentOk, boolean expectedStalled) {
    MotorStallDetector det = makeDetector(1);
    FakeTelemetry t =
        new FakeTelemetry()
            .set(
                pickOutput(outputOk, true),
                pickVelocity(velocityOk, true),
                pickCurrent(currentOk));

    assertEquals(expectedStalled, det.isStalled(t));
  }

  @Test
  @DisplayName("Debounce: only stalls after N consecutive true samples")
  public void debounceWorks() {
    MotorStallDetector det = makeDetector(3);
    FakeTelemetry t = new FakeTelemetry().set(OUT_ABOVE_POS, VEL_WITHIN_POS, CUR_ABOVE);

    assertFalse(det.isStalled(t));
    assertFalse(det.isStalled(t));
    assertTrue(det.isStalled(t));

    assertEquals(3, det.getConsecutiveTrueLoops());
  }

  @Test
  @DisplayName("Exit stall immediately when condition clears")
  public void exitsStallImmediatelyWhenConditionClears() {
    MotorStallDetector det = makeDetector(3);
    FakeTelemetry t = new FakeTelemetry();

    t.set(OUT_ABOVE_POS, VEL_WITHIN_POS, CUR_ABOVE);
    assertFalse(det.isStalled(t));
    assertFalse(det.isStalled(t));
    assertTrue(det.isStalled(t));
    assertTrue(det.isStalled(t));

    t.set(OUT_ABOVE_POS, VEL_OUTSIDE_POS, CUR_ABOVE);
    assertFalse(det.isStalled(t));
    assertEquals(0, det.getConsecutiveTrueLoops());

    t.set(OUT_ABOVE_POS, VEL_WITHIN_POS, CUR_ABOVE);
    assertFalse(det.isStalled(t));
    assertFalse(det.isStalled(t));
    assertTrue(det.isStalled(t));
  }

  @Test
  @DisplayName("Alternating samples never accumulate")
  public void alternatingSamplesNeverStall() {
    MotorStallDetector det = makeDetector(3);
    FakeTelemetry t = new FakeTelemetry();

    for (int i = 0; i < 20; i++) {
      t.set(OUT_ABOVE_POS, VEL_WITHIN_POS, CUR_ABOVE);
      assertFalse(det.isStalled(t));

      t.set(OUT_ABOVE_POS, VEL_WITHIN_POS, CUR_BELOW);
      assertFalse(det.isStalled(t));
      assertEquals(0, det.getConsecutiveTrueLoops());
    }
  }

  @Test
  @DisplayName("Telemetry getters are called exactly once per evaluation")
  public void callsTelemetryExactlyOncePerEvaluation() {
    MotorStallDetector det = makeDetector(1);
    FakeTelemetry t = new FakeTelemetry().set(OUT_ABOVE_POS, VEL_WITHIN_POS, CUR_ABOVE);

    t.resetCallCounts();
    det.isStalled(t);

    assertEquals(1, t.callsAppliedOutput);
    assertEquals(1, t.callsVelocity);
    assertEquals(1, t.callsCurrent);
  }

  @Test
  @DisplayName("Builder requires all fields")
  public void builderRequiresAllFields() {
    assertThrows(
        IllegalStateException.class, () -> MotorStallDetector.builder().requiredLoops(1).build());
  }

  @Test
  @DisplayName("Validation rejects bad values")
  public void validationRejectsBadValues() {
    assertThrows(
        IllegalArgumentException.class,
        () ->
            MotorStallDetector.builder()
                .minAppliedOutput(-0.1)
                .maxAbsVelocity(0.0)
                .minCurrentAmps(0.0)
                .requiredLoops(1)
                .build());

    assertThrows(
        IllegalArgumentException.class,
        () ->
            MotorStallDetector.builder()
                .minAppliedOutput(0.2)
                .maxAbsVelocity(-0.01)
                .minCurrentAmps(0.0)
                .requiredLoops(1)
                .build());

    assertThrows(
        IllegalArgumentException.class,
        () ->
            MotorStallDetector.builder()
                .minAppliedOutput(0.2)
                .maxAbsVelocity(0.0)
                .minCurrentAmps(-5.0)
                .requiredLoops(1)
                .build());

    assertThrows(
        IllegalArgumentException.class,
        () ->
            MotorStallDetector.builder()
                .minAppliedOutput(0.2)
                .maxAbsVelocity(0.0)
                .minCurrentAmps(5.0)
                .requiredLoops(0)
                .build());
  }
}
