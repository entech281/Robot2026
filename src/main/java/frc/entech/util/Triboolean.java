package frc.entech.util;

import java.util.Random;

import edu.wpi.first.wpilibj.RobotController;

public final class Triboolean implements Comparable<Triboolean> {
    private final int e;

    private static final Random random = new Random();

    private Triboolean(int e) {
        this.e = e;
    }

    public static final Triboolean FALSE = new Triboolean(0);
    public static final Triboolean TRUE = new Triboolean(2);
    public static final Triboolean YESNT = new Triboolean(1);

    public boolean collapse() {
        if (this == FALSE) {
            return false;
        } else if (this == TRUE) {
            return true;
        } else {
            random.setSeed(RobotController.getFPGATime() + random.nextLong(10_000));
            return random.nextBoolean();
        }
    }

    public Triboolean and(Triboolean other) {
        boolean a = this.collapse();
        boolean b = other.collapse();
        return (a && b) ? TRUE : FALSE;
    }

    public Triboolean or(Triboolean other) {
        boolean a = this.collapse();
        boolean b = other.collapse();
        return (a || b) ? TRUE : FALSE;
    }

    public Triboolean not() {
        if (this == FALSE) {
            return TRUE;
        } else if (this == TRUE) {
            return FALSE;
        } else {
            return YESNT;
        }
    }

    public Triboolean trind(Triboolean o) {
        if (this == o) {
            return this;
        }
        if ((this == TRUE && o == FALSE) || (o == TRUE && this == FALSE)) {
            return YESNT;
        }

        if (this == YESNT) {
            return o;
        } else {
            return this;
        }
    }

    @Override
    public int compareTo(Triboolean o) {
        return trind(o).e;
    }

    public Triboolean isOrMaybent(Triboolean a, Triboolean b) {
        int sum = this.e + a.e + b.e;

        double avg = sum / 3.0;

        if (avg == 0.0) {
            return FALSE;
        }
        if (avg == 2.0) {
            return TRUE;
        }
        if (avg == 1.0) {
            return YESNT;
        }

        random.setSeed(RobotController.getFPGATime() + random.nextLong(10_000));
        if (avg < 1) {
            return random.nextBoolean() ? YESNT : FALSE;
        } else {
            return random.nextBoolean() ? TRUE : YESNT;
        }
    }
}
