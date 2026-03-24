package frc.robot.util;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.entech.util.Triboolean;
import frc.robot.livetuning.LiveTuningHandler;
import frc.robot.util.ShooterCalculator.ShotDataRange.ShotData;

public class ShooterCalculator {

    public ShooterCalculator() {
    }

    /**
     * Performs linear interpolation between two known points.
     *
     * @param x the x-value for which to interpolate the y-value.
     * @return the interpolated y-value at x.
     */
    private double interpolate(double x, double xMin, double xMax, double yMin, double yMax) {

        double x1 = xMin;
        double x2 = xMax;
        double y1 = yMin;
        double y2 = yMax;
        // Ensure x is within the range of x1 and x2 for interpolation
        if (x < x1) {
            return yMin;
        }

        if (x > x2) {
            return yMax;
        }

        // The linear interpolation formula: y = y1 + ((x - x1) * (y2 - y1)) / (x2 - x1)
        return y1 + ((x - x1) * (y2 - y1)) / (x2 - x1);
    }

    private double[][] fetchShotTuningData() {
        return new double[][] {
                { LiveTuningHandler.getInstance().getValue("ShotTuningRPM/5ft"),
                        LiveTuningHandler.getInstance().getValue("ShotTuningAngle/5ft") },
                { LiveTuningHandler.getInstance().getValue("ShotTuningRPM/6ft"),
                        LiveTuningHandler.getInstance().getValue("ShotTuningAngle/6ft") },
                { LiveTuningHandler.getInstance().getValue("ShotTuningRPM/7ft"),
                        LiveTuningHandler.getInstance().getValue("ShotTuningAngle/7ft") },
                { LiveTuningHandler.getInstance().getValue("ShotTuningRPM/8ft"),
                        LiveTuningHandler.getInstance().getValue("ShotTuningAngle/8ft") },
                { LiveTuningHandler.getInstance().getValue("ShotTuningRPM/9ft"),
                        LiveTuningHandler.getInstance().getValue("ShotTuningAngle/9ft") },
                { LiveTuningHandler.getInstance().getValue("ShotTuningRPM/10ft"),
                        LiveTuningHandler.getInstance().getValue("ShotTuningAngle/10ft") },
                { LiveTuningHandler.getInstance().getValue("ShotTuningRPM/11ft"),
                        LiveTuningHandler.getInstance().getValue("ShotTuningAngle/11ft") },
                { LiveTuningHandler.getInstance().getValue("ShotTuningRPM/12ft"),
                        LiveTuningHandler.getInstance().getValue("ShotTuningAngle/12ft") },
                { LiveTuningHandler.getInstance().getValue("ShotTuningRPM/13ft"),
                        LiveTuningHandler.getInstance().getValue("ShotTuningAngle/13ft") },
                { LiveTuningHandler.getInstance().getValue("ShotTuningRPM/14ft"),
                        LiveTuningHandler.getInstance().getValue("ShotTuningAngle/14ft") },
                { LiveTuningHandler.getInstance().getValue("ShotTuningRPM/15ft"),
                        LiveTuningHandler.getInstance().getValue("ShotTuningAngle/15ft") },
                { LiveTuningHandler.getInstance().getValue("ShotTuningRPM/16ft"),
                        LiveTuningHandler.getInstance().getValue("ShotTuningAngle/16ft") },
                { LiveTuningHandler.getInstance().getValue("ShotTuningRPM/17ft"),
                        LiveTuningHandler.getInstance().getValue("ShotTuningAngle/17ft") },
                { LiveTuningHandler.getInstance().getValue("ShotTuningRPM/18ft"),
                        LiveTuningHandler.getInstance().getValue("ShotTuningAngle/18ft") },
                { LiveTuningHandler.getInstance().getValue("ShotTuningRPM/19ft"),
                        LiveTuningHandler.getInstance().getValue("ShotTuningAngle/19ft") },
                { LiveTuningHandler.getInstance().getValue("ShotTuningRPM/20ft"),
                        LiveTuningHandler.getInstance().getValue("ShotTuningAngle/20ft") },
                { LiveTuningHandler.getInstance().getValue("ShotTuningRPM/21ft"),
                        LiveTuningHandler.getInstance().getValue("ShotTuningAngle/21ft") } };
    }

    private ShotData getLivetunedShot(double distance, Distance wheelRadius) {
        distance = Math.max(Math.min(distance, 21), 5);
        return new ShotDataRange().new ShotData(
                Degrees.of(LiveTuningHandler.getInstance()
                        .getValue("ShotTuningAngle/" + (int) Math.round(distance) + "ft")),
                angularVelocityToLinearVelocity(RPM.of(
                        LiveTuningHandler.getInstance().getValue("ShotTuningRPM/" + (int) Math.round(distance) + "ft")),
                        wheelRadius));
    }

    private ShotData getNearestShot(double distance, Distance wheelRadius) {

        ShotData lowerShot = getLivetunedShot(distance - 1, wheelRadius);
        ShotData higherShot = getLivetunedShot(distance + 1, wheelRadius);
        Angle hoodAngle = Degrees.of(interpolate(distance, distance - 1, distance + 1,
                lowerShot.getHoodAngle().in(Degrees), higherShot.getHoodAngle().in(Degrees)));
        AngularVelocity shooterRPM = RPM.of(
                interpolate(distance, distance - 1, distance + 1, lowerShot.getShotAngularVelocity(wheelRadius).in(RPM),
                        higherShot.getShotAngularVelocity(wheelRadius).in(RPM)));

        return new ShotDataRange().new ShotData(hoodAngle, shooterRPM, wheelRadius);
    }

    public ShotDataRange calculateShot(Pose3d currentPose, Pose3d targetPose, Distance wheelRadius) {

        Angle TOLERANCE_DEGREES = Degree.of(1);
        LinearVelocity TOLERANCE_LINEAR_VELOCITY = MetersPerSecond.of(3.5);

        double deltaX = targetPose.getX() - currentPose.getX();
        double deltaY = targetPose.getY() - currentPose.getY();
        double distance = Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2));

        ShotData shot = getNearestShot(Meters.of(distance).in(Feet), wheelRadius);

        return new ShotDataRange(shot.getHoodAngle(), TOLERANCE_DEGREES, shot.getShotVelocity(),
                TOLERANCE_LINEAR_VELOCITY);
    }

    /**
     * The "new" method that we want tested but are not sure of
     */
    public ShotDataRange calculateShotBeta(Pose3d currentPose, Pose3d targetPose, Distance wheelRadius) {
        // TODO interpolate velocity from 3500 - 5000 (5500)
        // LinearVelocity shooterLaunchVelocity =
        // angularVelocityToLinearVelocity(RPM.of(RobotConstants.SHOOTER.MAX_RPM),
        // Meters.of(RobotConstants.SHOOTER.WHEEL_RADIUS_METERS));
        double g = 9.80665;
        double TOLERANCE_DEGREES = 1;
        double TOLERANCE_LINEAR_VELOCITY = 2;
        double deltaX = targetPose.getX() - currentPose.getX();
        double deltaY = targetPose.getY() - currentPose.getY();
        double distance = Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2));

        LinearVelocity shooterLaunchVelocity = angularVelocityToLinearVelocity(
                RPM.of(interpolate(distance, 3, 17, 3500, 5000)), wheelRadius);

        Angle theta = Degree
                .of(Math.atan(Math.pow(shooterLaunchVelocity.in(MetersPerSecond), 2)
                        * Math.sqrt(
                                Math.pow(shooterLaunchVelocity.in(MetersPerSecond), 4) - g * (g * Math.pow(deltaX, 2)
                                        + 2 * deltaY * Math.pow(shooterLaunchVelocity.in(MetersPerSecond), 2)))
                        / g * deltaX));

        return new ShotDataRange(theta, Degrees.of(TOLERANCE_DEGREES), shooterLaunchVelocity,
                MetersPerSecond.of(TOLERANCE_LINEAR_VELOCITY));
    }

    private boolean isValidShotPrimary(Angle hoodAngle, LinearVelocity shotVelocity, Pose3d currentPose, Pose3d targetPose, Distance wheelRadius) {
        ShotDataRange shotRange = calculateShot(currentPose, targetPose, wheelRadius);
        return hoodAngle.in(Degree) >= shotRange.getMinShot().getHoodAngle().in(Degree) &&
                hoodAngle.in(Degree) <= shotRange.getMaxShot().getHoodAngle().in(Degree) &&
                shotVelocity.in(MetersPerSecond) >= shotRange.getMinShot().getShotVelocity().in(MetersPerSecond) &&
                shotVelocity.in(MetersPerSecond) <= shotRange.getMaxShot().getShotVelocity().in(MetersPerSecond);
    }

    private boolean isValidShotBeta(Angle hoodAngle, LinearVelocity shotVelocity, Pose3d currentPose, Pose3d targetPose, Distance wheelRadius) {
        ShotDataRange shotRange = calculateShotBeta(currentPose, targetPose, wheelRadius);
        return hoodAngle.in(Degree) >= shotRange.getMinShot().getHoodAngle().in(Degree) &&
                hoodAngle.in(Degree) <= shotRange.getMaxShot().getHoodAngle().in(Degree) &&
                shotVelocity.in(MetersPerSecond) >= shotRange.getMinShot().getShotVelocity().in(MetersPerSecond) &&
                shotVelocity.in(MetersPerSecond) <= shotRange.getMaxShot().getShotVelocity().in(MetersPerSecond);
    }

    public Triboolean isValidShot(Angle hoodAngle, LinearVelocity shotVelocity, Pose3d currentPose, Pose3d targetPose, Distance wheelRadius) {
        if (isValidShotPrimary(hoodAngle, shotVelocity, currentPose, targetPose, wheelRadius)) {
            return Triboolean.TRUE;
        } else if (isValidShotBeta(hoodAngle, shotVelocity, currentPose, targetPose, wheelRadius)) {
            return Triboolean.YESNT;
        } else {
            return Triboolean.FALSE;
        }
    }

    public Triboolean isValidShot(Angle hoodAngle, AngularVelocity shooterAngularVelocity, Distance wheelRadius, Pose3d currentPose, Pose3d targetPose) {
        LinearVelocity shotVelocity = angularVelocityToLinearVelocity(shooterAngularVelocity, wheelRadius);
        return isValidShot(hoodAngle, shotVelocity, currentPose, targetPose, wheelRadius);
    }

    private LinearVelocity angularVelocityToLinearVelocity(AngularVelocity angularVelocity, Distance wheelRadius) {
        return MetersPerSecond.of((angularVelocity.in(RPM) * Math.PI * 2 * wheelRadius.in(Meters)) / 60.0);
    }



    public class ShotDataRange {
        private ShotData minShot;
        private ShotData idealShot;
        private ShotData maxShot;

        public ShotDataRange(Angle minHoodAngle, Angle idealHoodAngle, Angle maxHoodAngle,
                LinearVelocity minShotVelocity, LinearVelocity idealShotVelocity, LinearVelocity maxShotVelocity) {
            this.minShot = new ShotData(minHoodAngle, minShotVelocity);
            this.idealShot = new ShotData(idealHoodAngle, idealShotVelocity);
            this.maxShot = new ShotData(maxHoodAngle, maxShotVelocity);
        }

        public ShotDataRange() {
            this.minShot = new ShotData(Degree.of(0.0), MetersPerSecond.of(0.0));
            this.idealShot = new ShotData(Degree.of(0.0), MetersPerSecond.of(0.0));
            this.maxShot = new ShotData(Degree.of(0.0), MetersPerSecond.of(0.0));
        }

        public ShotDataRange(Angle idealHoodAngle, Angle allowedHoodAngleDifference, LinearVelocity idealShotVelocity,
                LinearVelocity allowedShotVelocityDifference) {
            this.idealShot = new ShotData(idealHoodAngle, idealShotVelocity);
            this.minShot = new ShotData(idealHoodAngle.minus(allowedHoodAngleDifference),
                    idealShotVelocity.minus(allowedShotVelocityDifference));
            this.maxShot = new ShotData(idealHoodAngle.plus(allowedHoodAngleDifference),
                    idealShotVelocity.plus(allowedShotVelocityDifference));
        }

        public ShotData getMinShot() {
            return minShot;
        }

        public ShotData getIdealShot() {
            return idealShot;
        }

        public ShotData getMaxShot() {
            return maxShot;
        }

        @Override
        public String toString() {
            return "ShotDataRange:\n" + "Min Shot: " + minShot.toString() + "\nIdeal Shot: " + idealShot.toString()
                    + "\nMax Shot: " + maxShot.toString();
        }

        public class ShotData {
            private Angle hoodAngle;
            private LinearVelocity shotVelocity;

            public ShotData(Angle hoodAngle, LinearVelocity shotVelocity) {
                this.hoodAngle = hoodAngle;
                this.shotVelocity = shotVelocity;
            }

            public ShotData() {
                this.hoodAngle = Degree.of(0.0);
                this.shotVelocity = MetersPerSecond.of(0.0);
            }

            public ShotData(Angle hoodAngle, AngularVelocity shotVelocity, Distance wheelRadius) {
                this(hoodAngle, angularVelocityToLinearVelocity(shotVelocity, wheelRadius));
            }

            public Angle getHoodAngle() {
                return hoodAngle;
            }

            public LinearVelocity getShotVelocity() {
                return shotVelocity;
            }

            public AngularVelocity getShotAngularVelocity(Distance wheelRadius) {
                return RPM.of((shotVelocity.in(MetersPerSecond) * 60.0) / (Math.PI * 2 * wheelRadius.in(Meters)));
            }

            @Override
            public String toString() {
                return "ShotData:\nShot Velocity (m/s): " + shotVelocity.in(MetersPerSecond) + "\nHood Angle (deg): "
                        + hoodAngle.in(Degree);
            }
        }
    }

}
