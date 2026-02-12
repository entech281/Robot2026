package frc.robot.util;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

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

    public ShotData calculateShot() {
        return new ShotData();
    }

    public void refresh(ChassisSpeeds robotVelocity, Pose3d currentPose, Pose3d targetPose) {
        this.robotVelocity = robotVelocity;
        this.currentPose = currentPose;
        this.targetPose = targetPose;
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
    }
    
}
