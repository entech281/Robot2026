package frc.robot.util;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
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
import frc.robot.RobotConstants;

public class ShooterCalculator {

    private ChassisSpeeds robotVelocity;
    private Pose3d currentPose;
    private Pose3d targetPose;

    public ShooterCalculator() {
        this.robotVelocity = new ChassisSpeeds();
        this.currentPose = new Pose3d();
        this.targetPose = new Pose3d();
    }

    public ShooterCalculator(ChassisSpeeds robotVelocity, Pose3d currentPose, Pose3d targetPose) {
        this.robotVelocity = robotVelocity;
        this.currentPose = currentPose;
        this.targetPose = targetPose;
    }

    public ShotDataRange calculateShot() {
        LinearVelocity shooterLaunchVelocity = angularVelocityToLinearVelocity(RPM.of(RobotConstants.SHOOTER.MAX_RPM), Meters.of(RobotConstants.SHOOTER.WHEEL_RADIUS_METERS));
        double g = 9.80665;
        double TOLERANCE_DEGREES = 1;
        double TOLERANCE_LINEAR_VELOCITY = 2;
        double deltaX = currentPose.getTranslation().getDistance(targetPose.getTranslation());
        double deltaY = targetPose.getZ() - currentPose.getZ();
        Angle theta = Degree.of(Math.atan(Math.pow(shooterLaunchVelocity.in(MetersPerSecond), 2) * Math.sqrt(Math.pow(shooterLaunchVelocity.in(MetersPerSecond), 4) - g * (g * Math.pow(deltaX, 2) + 2 * deltaY * Math.pow(shooterLaunchVelocity.in(MetersPerSecond), 2))) / g * deltaX));

        return new ShotDataRange(theta, Degrees.of(TOLERANCE_DEGREES), shooterLaunchVelocity, MetersPerSecond.of(TOLERANCE_LINEAR_VELOCITY));
    }

    /**
     * The "new" method that we want tested but are not sure of
     */
    public ShotDataRange calculateShotBeta () {
        //TODO: same as primary now until new formula
        return calculateShot();
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
        return MetersPerSecond.of( (angularVelocity.in(RPM) * Math.PI * 2 * wheelRadius.in(Meters)) / 60.0 );
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

        public ShotDataRange(Angle minHoodAngle, Angle idealHoodAngle, Angle maxHoodAngle, LinearVelocity minShotVelocity, LinearVelocity idealShotVelocity, LinearVelocity maxShotVelocity) {
            this.minShot = new ShotData(minHoodAngle, minShotVelocity);
            this.idealShot = new ShotData(idealHoodAngle, idealShotVelocity);
            this.maxShot = new ShotData(maxHoodAngle, maxShotVelocity);
        }

        public ShotDataRange() {
            this.minShot = new ShotData(Degree.of(0.0), MetersPerSecond.of(0.0));
            this.idealShot = new ShotData(Degree.of(0.0), MetersPerSecond.of(0.0));
            this.maxShot = new ShotData(Degree.of(0.0), MetersPerSecond.of(0.0));
        }

        public ShotDataRange(Angle idealHoodAngle, Angle allowedHoodAngleDifference, LinearVelocity idealShotVelocity, LinearVelocity allowedShotVelocityDifference) {
            this.idealShot = new ShotData(idealHoodAngle, idealShotVelocity);
            this.minShot = new ShotData(idealHoodAngle.minus(allowedHoodAngleDifference), idealShotVelocity.minus(allowedShotVelocityDifference));
            this.maxShot = new ShotData(idealHoodAngle.plus(allowedHoodAngleDifference), idealShotVelocity.plus(allowedShotVelocityDifference));
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
            return "ShotDataRange:\n" + "Min Shot: " + minShot.toString() + "\nIdeal Shot: " + idealShot.toString() + "\nMax Shot: " + maxShot.toString();
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

            public Angle getHoodAngle() {
                return hoodAngle;
            }

            public LinearVelocity getShotVelocity() {
                return shotVelocity;
            }

            public AngularVelocity getShotAngularVelocity(Distance wheelRadius) {
                return RPM.of( (shotVelocity.in(MetersPerSecond) * 60.0) / (Math.PI * 2 * wheelRadius.in(Meters)) );
            }

            @Override
            public String toString() {
                return "ShotData:\nShot Velocity (m/s): " + shotVelocity.in(MetersPerSecond) + "\nHood Angle (deg): " + hoodAngle.in(Degree);
            }
        }   
    }
    
}
