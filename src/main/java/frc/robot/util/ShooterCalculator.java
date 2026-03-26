package frc.robot.util;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.entech.util.Triboolean;
import frc.robot.livetuning.LiveTuningHandler;
import frc.robot.util.ShooterCalculator.ShotDataRange.ShotData;

public class ShooterCalculator {

    private ChassisSpeeds robotVelocity;
    private Pose3d currentPose;
    private Pose3d targetPose;
    private Distance wheelRadius;
    private AngularVelocity shooterMax;
    private AngularVelocity shooterMin;
    private Distance distanceMin;
    private Distance distanceMax;

    public ShooterCalculator() {
        this(new ChassisSpeeds(), new Pose3d(), new Pose3d(), Meters.of(1), RPM.of(0), RPM.of(0), Meters.of(0),
                Meters.of(0));
    }

    public ShooterCalculator(ChassisSpeeds robotVelocity, Pose3d currentPose, Pose3d targetPose, Distance wheelRadius,
            AngularVelocity shooterMax, AngularVelocity shooterMin, Distance distanceMax, Distance distanceMin) {

        if (wheelRadius.in(Meters) <= 0) {
            throw new IllegalArgumentException("wheelRadius cannot be zero or less than zero");
        }

        this.robotVelocity = robotVelocity;
        this.currentPose = currentPose;
        this.targetPose = targetPose;
        this.wheelRadius = wheelRadius;
        this.shooterMax = shooterMax;
        this.shooterMin = shooterMin;
        this.distanceMax = distanceMax;
        this.distanceMin = distanceMin;
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

    private ShotData getLivetunedShot(double distance) {
        distance = Math.max(Math.min(distance, 21), 5);
        return new ShotDataRange().new ShotData(
                Degrees.of(LiveTuningHandler.getInstance()
                        .getValue("ShotTuningAngle/" + (int) Math.round(distance) + "ft")),
                angularVelocityToLinearVelocity(RPM.of(
                        LiveTuningHandler.getInstance().getValue("ShotTuningRPM/" + (int) Math.round(distance) + "ft")),
                        wheelRadius));
    }

    private ShotData getNearestShot(double distance) {

        ShotData lowerShot = getLivetunedShot(distance - 1);
        ShotData higherShot = getLivetunedShot(distance + 1);
        Angle hoodAngle = Degrees.of(interpolate(distance, distance - 1, distance + 1,
                lowerShot.getHoodAngle().in(Degrees), higherShot.getHoodAngle().in(Degrees)));
        AngularVelocity shooterRPM = RPM.of(
                interpolate(distance, distance - 1, distance + 1, lowerShot.getShotAngularVelocity(wheelRadius).in(RPM),
                        higherShot.getShotAngularVelocity(wheelRadius).in(RPM)));

        return new ShotDataRange().new ShotData(hoodAngle, shooterRPM, wheelRadius);
    }

    public ShotDataRange calculateShot() {

        Angle TOLERANCE_DEGREES = Degree.of(1);
        LinearVelocity TOLERANCE_LINEAR_VELOCITY = MetersPerSecond.of(3.5);

        double deltaX = targetPose.getX() - currentPose.getX();
        double deltaY = targetPose.getY() - currentPose.getY();
        double distance = Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2));

        ShotData shot = getNearestShot(Meters.of(distance).in(Feet));

        return new ShotDataRange(shot.getHoodAngle(), TOLERANCE_DEGREES, shot.getShotVelocity(),
                TOLERANCE_LINEAR_VELOCITY);
    }

    /**
     * Compute the component of robot velocity along the robot→target axis.
     * Positive = moving toward target, negative = moving away.
     */
    private double computeRadialVelocityMps(double deltaX, double deltaY, double distance) {
        if (distance < 0.1) {
            return 0.0;
        }
        double unitX = deltaX / distance;
        double unitY = deltaY / distance;

        ChassisSpeeds fieldVelocity = ChassisSpeeds.fromRobotRelativeSpeeds(
                robotVelocity,
                currentPose.toPose2d().getRotation());

        return fieldVelocity.vxMetersPerSecond * unitX + fieldVelocity.vyMetersPerSecond * unitY;
    }

    /** Get the radial velocity in m/s for logging. Positive = moving toward target. */
    public double getRadialVelocityMps() {
        double deltaX = targetPose.getX() - currentPose.getX();
        double deltaY = targetPose.getY() - currentPose.getY();
        double distance = Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2));
        return computeRadialVelocityMps(deltaX, deltaY, distance);
    }

    /**
     * Motion-compensated shooting: accounts for robot velocity toward/away from hub.
     * Uses flight time estimates from pyshooter to compute an effective distance,
     * then looks up RPM and hood angle at that effective distance.
     */
    public ShotDataRange calculateShotBeta() {

        Angle TOLERANCE_DEGREES = Degree.of(1);
        LinearVelocity TOLERANCE_LINEAR_VELOCITY = MetersPerSecond.of(3.5);

        double deltaX = targetPose.getX() - currentPose.getX();
        double deltaY = targetPose.getY() - currentPose.getY();
        double distanceMeters = Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2));
        double distanceFeet = Meters.of(distanceMeters).in(Feet);

        // First lookup at actual distance to get hood angle for flight time estimate
        ShotData baseShot = getNearestShot(distanceFeet);
        double hoodAngleDeg = baseShot.getHoodAngle().in(Degrees);

        // Compute radial velocity (robot motion toward/away from hub)
        double radialVelocityMps = computeRadialVelocityMps(deltaX, deltaY, distanceMeters);

        // Compute effective distance: the ball inherits the robot's radial velocity,
        // so moving toward the hub is equivalent to shooting at a shorter distance.
        // This lets the tuning table naturally correct both RPM and hood angle.
        double flightTime = FlightTimeEstimator.getFlightTimeSeconds(distanceMeters, hoodAngleDeg);
        double effectiveDistanceFeet = distanceFeet - Meters.of(radialVelocityMps * flightTime).in(Feet);

        // Clamp to tuning table range
        effectiveDistanceFeet = Math.max(5.0, Math.min(21.0, effectiveDistanceFeet));

        ShotData shot = getNearestShot(effectiveDistanceFeet);

        return new ShotDataRange(shot.getHoodAngle(), TOLERANCE_DEGREES, shot.getShotVelocity(),
                TOLERANCE_LINEAR_VELOCITY);
    }

    private boolean isValidShotPrimary(Angle hoodAngle, LinearVelocity shotVelocity) {
        ShotDataRange shotRange = calculateShot();
        return hoodAngle.in(Degree) >= shotRange.getMinShot().getHoodAngle().in(Degree) &&
                hoodAngle.in(Degree) <= shotRange.getMaxShot().getHoodAngle().in(Degree) &&
                shotVelocity.in(MetersPerSecond) >= shotRange.getMinShot().getShotVelocity().in(MetersPerSecond) &&
                shotVelocity.in(MetersPerSecond) <= shotRange.getMaxShot().getShotVelocity().in(MetersPerSecond);
    }

    private boolean isValidShotBeta(Angle hoodAngle, LinearVelocity shotVelocity) {
        ShotDataRange shotRange = calculateShotBeta();
        return hoodAngle.in(Degree) >= shotRange.getMinShot().getHoodAngle().in(Degree) &&
                hoodAngle.in(Degree) <= shotRange.getMaxShot().getHoodAngle().in(Degree) &&
                shotVelocity.in(MetersPerSecond) >= shotRange.getMinShot().getShotVelocity().in(MetersPerSecond) &&
                shotVelocity.in(MetersPerSecond) <= shotRange.getMaxShot().getShotVelocity().in(MetersPerSecond);
    }

    public Triboolean isValidShot(Angle hoodAngle, LinearVelocity shotVelocity) {
        if (isValidShotPrimary(hoodAngle, shotVelocity)) {
            return Triboolean.TRUE;
        } else if (isValidShotBeta(hoodAngle, shotVelocity)) {
            return Triboolean.YESNT;
        } else {
            return Triboolean.FALSE;
        }
    }

    public Triboolean isValidShot(Angle hoodAngle, AngularVelocity shooterAngularVelocity, Distance wheelRadius) {
        LinearVelocity shotVelocity = angularVelocityToLinearVelocity(shooterAngularVelocity, wheelRadius);
        return isValidShot(hoodAngle, shotVelocity);
    }

    private LinearVelocity angularVelocityToLinearVelocity(AngularVelocity angularVelocity, Distance wheelRadius) {
        return MetersPerSecond.of((angularVelocity.in(RPM) * Math.PI * 2 * wheelRadius.in(Meters)) / 60.0);
    }

    public void refresh(ChassisSpeeds robotVelocity, Pose3d currentPose, Pose3d targetPose) {
        this.robotVelocity = robotVelocity;
        this.currentPose = currentPose;
        this.targetPose = targetPose;
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
