package frc.robot.util;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class ShooterCalculator {

    public ShotData calculateShot(ChassisSpeeds robotVelocity, Pose3d currentPose, Pose3d targetPose) {
        return new ShotData();
    }

    public class ShotData {
        private double hoodAngle;
        private double shooterRPM;

        public ShotData(double hoodAngle, double shooterRPM) {
            this.hoodAngle = hoodAngle;
            this.shooterRPM = shooterRPM;
        }

        public ShotData() {
            this.hoodAngle = 0.0;
            this.shooterRPM = 0.0;
        }

        public double getHoodAngle() {
            return hoodAngle;
        }

        public double getShooterRPM() {
            return shooterRPM;
        }
    }
    
}
