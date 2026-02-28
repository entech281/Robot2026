package frc.robot.util;

import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

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

    public ShiftStateTracker(double warningSeconds) {
        this.wonAuto = getFirstActiveAlliance();
        this.warningSeconds = warningSeconds;
    }

    public ShiftStateTracker(boolean wonAuto, double warningSeconds) {
        this.wonAuto = wonAuto;
        this.warningSeconds = warningSeconds;
    }



  public static boolean areWeFirstAlliance() {
    var alliance = getAlliance();
            // Return FMS value
        boolean first = getFirstActiveAlliance();
        String message = getGameSpecificMessage();
        if (message.length() > 0) {
            if (first) {
                return true;
            }
            else return false;  
        }
            return false;
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
    
    public static String getAlliance(){
        if (DriverStation.getAlliance().equals(Alliance.Red)) {
            return "Red";
        } else if (DriverStation.getAlliance().equals(Alliance.Blue)) {
            return "Blue";
        } else {
            return "Invalid Alliance";
        }
    }

    private static boolean getFirstActiveAlliance() {
        String message = DriverStation.getGameSpecificMessage();
        if (message.length() > 0) {
            char character = message.charAt(0);
            if (character == 'R') {
                return true;
            }
            else if (character == 'B') {
                return false;
            }
        }
        // Default to true if we can't get a message, but log an error
        System.err.println("Could not get game-specific message to determine alliance! Defaulting to Red.");
        return true;
    }

    public static String getGameSpecificMessage(){
        return DriverStation.getGameSpecificMessage();
    }

    public double getWarningSeconds() {
        return warningSeconds;
    }
}