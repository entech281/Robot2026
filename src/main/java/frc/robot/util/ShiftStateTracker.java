// ShiftStateTracker.java
package frc.robot.util;

/**
 * Tracks 2026 FRC REBUILT shift states and provides LED-ready state enum.
 *
 * Match time counts DOWN from 140s (start of teleop) to 0s.
 *
 * Shift schedule (matchTime remaining):
 *   140-130: Transition  — both hubs active
 *   130-105: Shift 1     — Auto LOSER  active
 *   105- 80: Shift 2     — Auto WINNER active
 *    80- 55: Shift 3     — Auto LOSER  active
 *    55- 30: Shift 4     — Auto WINNER active
 *    30-  0: End Game    — both hubs active
 *
 * Winner (scored MORE in auto):
 *   Transition is YOUR_SHIFT (with SHIFT_ENDING warning near 130)
 *   Shift 1 is THEIR_SHIFT (with SHIFT_STARTING warning near 105)
 *   Shift 2, 4, EndGame are YOUR_SHIFT
 *
 * Loser (scored LESS in auto):
 *   Transition flows straight into Shift 1 — no warning needed
 *   Shift 1, 3, EndGame are YOUR_SHIFT
 *   Shift 2 is THEIR_SHIFT (with SHIFT_STARTING warning near 80)
 */
public class ShiftStateTracker {

    public enum ShiftState {
        YOUR_SHIFT,     // Green  — your hub is active
        SHIFT_ENDING,   // Yellow — your shift ends within N seconds
        THEIR_SHIFT,    // Red    — opponent's hub is active
        SHIFT_STARTING  // Yellow — your next shift starts within N seconds
    }

    // Winner (scored MORE): Transition + Shifts 2, 4, EndGame
    private static final double[][] WINNER_SHIFTS = {
        {140.0, 130.0}, // Transition
        {105.0,  80.0}, // Shift 2
        { 55.0,  30.0}, // Shift 4
        { 30.0,   0.0}  // End Game
    };

    // Loser (scored LESS): Shifts 1, 3, EndGame (no transition — flows straight in)
    private static final double[][] LOSER_SHIFTS = {
        {130.0, 105.0}, // Shift 1
        { 80.0,  55.0}, // Shift 3
        { 30.0,   0.0}  // End Game
    };

    private static final double TELEOP_START = 140.0;

    private final boolean wonAuto;
    private final double warningSeconds;

    /**
     * @param wonAuto         true if your alliance scored MORE in Auto
     * @param warningSeconds  N — seconds before a shift starts/ends to show yellow warning
     */
    public ShiftStateTracker(boolean wonAuto, double warningSeconds) {
        this.wonAuto = wonAuto;
        this.warningSeconds = warningSeconds;
    }

    /**
     * Returns the current shift state given remaining teleop match time.
     *
     * @param matchTime remaining teleop time in seconds (140 → 0)
     * @return current ShiftState
     */
    public ShiftState getState(double matchTime) {
        if (matchTime > TELEOP_START || matchTime < 0.0) {
            return ShiftState.YOUR_SHIFT;
        }

        double[][] myShifts    = wonAuto ? WINNER_SHIFTS : LOSER_SHIFTS;
        double[][] theirShifts = wonAuto ? LOSER_SHIFTS  : WINNER_SHIFTS;

        // Are we currently in one of our shifts?
        for (double[] shift : myShifts) {
            double shiftStart = shift[0];
            double shiftEnd   = shift[1];
            if (matchTime <= shiftStart && matchTime > shiftEnd) {
                double timeLeftInShift = matchTime - shiftEnd;
                if (timeLeftInShift <= warningSeconds) {
                    return ShiftState.SHIFT_ENDING;
                }
                return ShiftState.YOUR_SHIFT;
            }
        }

        // We're in their shift — find time until our next shift starts
        double timeUntilOurNextShift = Double.MAX_VALUE;
        for (double[] shift : myShifts) {
            double shiftStart = shift[0];
            if (matchTime > shiftStart) {
                double timeUntil = matchTime - shiftStart;
                if (timeUntil < timeUntilOurNextShift) {
                    timeUntilOurNextShift = timeUntil;
                }
            }
        }

        if (timeUntilOurNextShift <= warningSeconds) {
            return ShiftState.SHIFT_STARTING;
        }
        return ShiftState.THEIR_SHIFT;
    }

    public boolean isWonAuto() {
        return wonAuto;
    }

    public double getWarningSeconds() {
        return warningSeconds;
    }
}