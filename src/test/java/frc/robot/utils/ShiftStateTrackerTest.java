package frc.robot.utils;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import frc.robot.util.ShiftStateTracker;

class ShiftStateTrackerTest {

    @Test
    void testTransitionPeriodEndingForWinner() {
        ShiftStateTracker tracker = new ShiftStateTracker(true, 5);

        assertEquals(
            ShiftStateTracker.ShiftState.SHIFT_ENDING,
            tracker.getState(134)

        );
    }


    @Test
    void testWinnerShiftActive() {
        ShiftStateTracker tracker = new ShiftStateTracker(true, 5);

        // Winner gets shift 2 (105–80)
        assertEquals(
            ShiftStateTracker.ShiftState.YOUR_SHIFT,
            tracker.getState(100)
        );

    }

    @Test
    void testWinnerShiftEndingWarning() {
        ShiftStateTracker tracker = new ShiftStateTracker(true, 5);

        // 3 seconds before 80
        assertEquals(
            ShiftStateTracker.ShiftState.SHIFT_ENDING, 
            tracker.getState(83));
        assertEquals( 
            ShiftStateTracker.ShiftState.SHIFT_ENDING, 
            tracker.getState(85));
    }

    @Test
    void testTheirShift() {
        ShiftStateTracker tracker = new ShiftStateTracker(true, 5);

        // Winner does NOT own shift 1 (130–105)
        assertEquals(
            ShiftStateTracker.ShiftState.THEIR_SHIFT,
            tracker.getState(120)
        );
        assertEquals(ShiftStateTracker.ShiftState.THEIR_SHIFT, tracker.getState(130));
    }

    @Test
    void testShiftStartingWarning() {
        ShiftStateTracker tracker = new ShiftStateTracker(true, 5);

        // 3 seconds before shift 2 begins at 105
        assertEquals(
            ShiftStateTracker.ShiftState.SHIFT_STARTING,
            tracker.getState(108)
        );
    }

    @Test
    void testEndGameAlwaysYourShift() {
        ShiftStateTracker tracker = new ShiftStateTracker(false, 5);

        assertEquals(
            ShiftStateTracker.ShiftState.YOUR_SHIFT,
            tracker.getState(20)
        );
    }

    @Test
    void testOutOfRangeSafeDefault() {
        ShiftStateTracker tracker = new ShiftStateTracker(true, 5);

        assertEquals(
            ShiftStateTracker.ShiftState.YOUR_SHIFT,
            tracker.getState(150)
        );

        assertEquals(
            ShiftStateTracker.ShiftState.YOUR_SHIFT,
            tracker.getState(-5)
        );
    }
}