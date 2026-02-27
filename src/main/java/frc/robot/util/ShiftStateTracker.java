package frc.robot.util;

public class ShiftStateTracker {

    public enum ShiftState {
        YOUR_SHIFT,
        SHIFT_ENDING,
        THEIR_SHIFT,
        SHIFT_STARTING
    }

    private static final double[][] WINNER_SHIFTS = {
        {140.0, 130.0}, // Transition
        {105.0,  80.0}, // Shift 2
        { 55.0,  30.0}, // Shift 4
        { 30.0,   0.0}  // End Game
    };

    private static final double[][] LOSER_SHIFTS = {
        {130.0, 105.0}, // Shift 1
        { 80.0,  55.0}, // Shift 3
        { 30.0,   0.0}  // End Game
    };

    private static final double TELEOP_START = 140.0;

    private final boolean wonAuto;
    private final double warningSeconds;

    public ShiftStateTracker(boolean wonAuto, double warningSeconds) {
        this.wonAuto = wonAuto;
        this.warningSeconds = warningSeconds;
    }

    public ShiftState getState(double matchTime) {
        if (matchTime > TELEOP_START || matchTime < 0.0) {
            return ShiftState.YOUR_SHIFT;
        }

        double[][] myShifts = wonAuto ? WINNER_SHIFTS : LOSER_SHIFTS;

        ShiftState activeShiftState = getActiveShiftState(matchTime, myShifts);
        if (activeShiftState != null) {
            return activeShiftState;
        }

        return getInactiveShiftState(matchTime, myShifts);
    }

    private ShiftState getActiveShiftState(double matchTime, double[][] myShifts) {
        for (double[] shift : myShifts) {
            double shiftStart = shift[0];
            double shiftEnd   = shift[1];
            if (matchTime <= shiftStart && matchTime > shiftEnd) {
                double timeLeftInShift = matchTime - shiftEnd;
                return timeLeftInShift <= warningSeconds
                    ? ShiftState.SHIFT_ENDING
                    : ShiftState.YOUR_SHIFT;
            }
        }
        return null;
    }

    private ShiftState getInactiveShiftState(double matchTime, double[][] myShifts) {
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
        return timeUntilOurNextShift <= warningSeconds
            ? ShiftState.SHIFT_STARTING
            : ShiftState.THEIR_SHIFT;
    }

    public boolean isWonAuto() {
        return wonAuto;
    }

    public double getWarningSeconds() {
        return warningSeconds;
    }
}