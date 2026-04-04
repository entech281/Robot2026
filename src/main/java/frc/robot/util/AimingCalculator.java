package frc.robot.util;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;

import java.awt.geom.Point2D;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import frc.robot.RobotConstants;
import frc.robot.livetuning.LiveTuningHandler;
import frc.robot.operation.UserPolicy;

public final class AimingCalculator {
    public enum VirtualPoseMode {
        DISABLED,
        ONESHOT,
        ITERATIVE
    }
    private AimingCalculator() {

    }

    /**
     * Calculates the necessary turret angle, hood angle, and shooter speed to aim
     * at a target.
     *
     * Pose3d Heights are used for some calculations but will not affect the height
     * aimed for.
     *
     * @param robotPose   The current pose of the robot.
     * @param targetPose  The pose of the target.
     * @param robotSpeeds The current speeds of the robot.
     * @return The necessary aiming data.
     */
    public static AimingOutputData calculateAimingData(Pose3d robotPose, Pose3d targetPose, ChassisSpeeds robotSpeeds) {
        LinearInterpolationTable flightTimeTable = getLiveFlightTimeTable();
        LinearInterpolationTable shooterTable = getLiveShooterTable();
        LinearInterpolationTable HoodTable = getLiveHoodTable();

        return calculateAimingData(robotPose, targetPose, robotSpeeds, flightTimeTable, shooterTable, HoodTable);
    }

    /**
     * Calculates the necessary turret angle, hood angle, and shooter speed to aim
     * at a target.
     *
     * Pose3d Heights are used for some calculations but will not affect the height
     * aimed for.
     *
     * @param robotPose   The current pose of the robot.
     * @param targetPose  The pose of the target.
     * @param robotSpeeds The current speeds of the robot.
     * @return The necessary aiming data.
     */
    public static AimingOutputData calculateAimingData(Pose3d robotPose, Pose3d targetPose, ChassisSpeeds robotSpeeds, LinearInterpolationTable flightTimeTable, LinearInterpolationTable shooterSpeedTable, LinearInterpolationTable hoodAngleTable) {
        ChassisSpeeds fieldAbsoluteSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(robotSpeeds, robotPose.toPose2d().getRotation());
        Pose3d virtualTarget = targetPose;
        if (UserPolicy.getInstance().getVirtualPoseMode() == VirtualPoseMode.ONESHOT) {
            Time flightTime = calculateFlightTime(robotPose, targetPose, flightTimeTable);
            virtualTarget = calculateVirtualPose(targetPose, fieldAbsoluteSpeeds, flightTime);
        } else if (UserPolicy.getInstance().getVirtualPoseMode() == VirtualPoseMode.ITERATIVE) {
            virtualTarget = calculateVirtualPoseIterative(robotPose, targetPose, fieldAbsoluteSpeeds, robotPose.toPose2d().getRotation(), flightTimeTable);
        }

        Angle turretAngle = calculateTurretAngle(robotPose.toPose2d(), targetPose.toPose2d()); //was virtual target
        if (UserPolicy.getInstance().isUseVirtualRotationCompensation()) {
            Time flightTime = calculateFlightTime(robotPose, virtualTarget, flightTimeTable);
            turretAngle = calculateVirtualAngleCompensation(turretAngle, fieldAbsoluteSpeeds, flightTime);
        }

        Angle hoodAngle = Degrees.of(hoodAngleTable.getOutput(calculateDistance(robotPose, virtualTarget).in(Feet)));

        AngularVelocity shooterSpeed = RPM.of(shooterSpeedTable.getOutput(calculateDistance(robotPose, virtualTarget).in(Feet)));

        return new AimingOutputData(turretAngle, hoodAngle, shooterSpeed);
    }

    public static Angle calculateTurretAngle(Pose2d robotPose, Pose2d virtualPose) {
        Translation2d turretToRobot = RobotConstants.TURRET.TURRET_OFFSET.rotateBy(robotPose.getRotation());
        Pose2d turretPose = new Pose2d(robotPose.getX() + turretToRobot.getX(),
                robotPose.getY() + turretToRobot.getY(),
                robotPose.getRotation());

        double angleToTarget = Math
                .toDegrees(Math.atan2(virtualPose.getY() - turretPose.getY(), virtualPose.getX() - turretPose.getX()));

        double fieldTurretAngle = angleToTarget - robotPose.getRotation().getDegrees();

        double turretAngleToTarget = (((-fieldTurretAngle) % 360) + 360) % 360;
        return Degrees.of(turretAngleToTarget);
    }

    public static Pose3d calculateVirtualPose(Pose3d targetPose, ChassisSpeeds speeds, Time flightTime) {
        double virtualPoseX = targetPose.getX() + (speeds.vxMetersPerSecond * flightTime.in(Seconds));
        double virtualPoseY = targetPose.getY() + (speeds.vyMetersPerSecond * flightTime.in(Seconds));
        Logger.recordOutput("virtualPose", new Pose3d(virtualPoseX, virtualPoseY, targetPose.getZ(), targetPose.getRotation()));
        return new Pose3d(virtualPoseX, virtualPoseY, targetPose.getZ(), targetPose.getRotation());
    }

    public static Pose3d calculateVirtualPoseIterative(Pose3d robotPose, Pose3d targetPose, ChassisSpeeds speeds, Rotation2d robotRotation, LinearInterpolationTable flightTimeLookup) {
        Time flightTime = Seconds.of(flightTimeLookup.getOutput(calculateDistance(robotPose, targetPose).in(Feet)));

        Pose3d virtualPose = targetPose;

        for (int i = 0; i < 5; i++) {
            Pose3d testVirtualPose = calculateVirtualPose(targetPose, speeds, flightTime);

            Time newFlightTime = calculateFlightTime(robotPose, targetPose, flightTimeLookup);

            if (Math.abs(newFlightTime.in(Seconds) - flightTime.in(Seconds)) <= 0.01) {
                i = 4;
            }

            if (i == 4) {
                virtualPose = testVirtualPose;
            } else {
                flightTime = newFlightTime;
            }
        }
        return virtualPose;
    }

    public static Angle calculateVirtualAngleCompensation(Angle turretAngle, ChassisSpeeds speeds, Time flightTime) {
        Angle modifiedAngle = turretAngle.minus(Radians.of(-speeds.omegaRadiansPerSecond * flightTime.in(Seconds)));
        return Degrees.of((((modifiedAngle.in(Degrees)) % 360) + 360) % 360);
    }

    public static Distance calculateDistance(Pose3d robot, Pose3d target) {
        double deltaX = target.getX() - robot.getX();
        double deltaY = target.getY() - robot.getY();
        return Meters.of(Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2)));
    }

    public static Time calculateFlightTime(Pose3d robotPose, Pose3d targetPose, LinearInterpolationTable flightTimeLookup) {
        return Seconds.of(flightTimeLookup.getOutput(calculateDistance(robotPose, targetPose).in(Feet)));
    }

    public static LinearInterpolationTable getLiveShooterTable(){
        return new LinearInterpolationTable(
            new Point2D.Double(5.0, LiveTuningHandler.getInstance().getValue("ShotTuningRPM/5ft")),
            new Point2D.Double(6.0, LiveTuningHandler.getInstance().getValue("ShotTuningRPM/6ft")),
            new Point2D.Double(7.0, LiveTuningHandler.getInstance().getValue("ShotTuningRPM/7ft")),
            new Point2D.Double(8.0, LiveTuningHandler.getInstance().getValue("ShotTuningRPM/8ft")),
            new Point2D.Double(9.0, LiveTuningHandler.getInstance().getValue("ShotTuningRPM/9ft")),
            new Point2D.Double(10.0, LiveTuningHandler.getInstance().getValue("ShotTuningRPM/10ft")),
            new Point2D.Double(11.0, LiveTuningHandler.getInstance().getValue("ShotTuningRPM/11ft")),
            new Point2D.Double(12.0, LiveTuningHandler.getInstance().getValue("ShotTuningRPM/12ft")),
            new Point2D.Double(13.0, LiveTuningHandler.getInstance().getValue("ShotTuningRPM/13ft")),
            new Point2D.Double(14.0, LiveTuningHandler.getInstance().getValue("ShotTuningRPM/14ft")),
            new Point2D.Double(15.0, LiveTuningHandler.getInstance().getValue("ShotTuningRPM/15ft")),
            new Point2D.Double(16.0, LiveTuningHandler.getInstance().getValue("ShotTuningRPM/16ft")),
            new Point2D.Double(17.0, LiveTuningHandler.getInstance().getValue("ShotTuningRPM/17ft")),
            new Point2D.Double(18.0, LiveTuningHandler.getInstance().getValue("ShotTuningRPM/18ft")),
            new Point2D.Double(19.0, LiveTuningHandler.getInstance().getValue("ShotTuningRPM/19ft")),
            new Point2D.Double(20.0, LiveTuningHandler.getInstance().getValue("ShotTuningRPM/20ft")),
            new Point2D.Double(21.0, LiveTuningHandler.getInstance().getValue("ShotTuningRPM/21ft"))
        );
    }

    public static LinearInterpolationTable getLiveHoodTable(){
        return new LinearInterpolationTable(
            new Point2D.Double(5.0, LiveTuningHandler.getInstance().getValue("ShotTuningAngle/5ft")),
            new Point2D.Double(6.0, LiveTuningHandler.getInstance().getValue("ShotTuningAngle/6ft")),
            new Point2D.Double(7.0, LiveTuningHandler.getInstance().getValue("ShotTuningAngle/7ft")),
            new Point2D.Double(8.0, LiveTuningHandler.getInstance().getValue("ShotTuningAngle/8ft")),
            new Point2D.Double(9.0, LiveTuningHandler.getInstance().getValue("ShotTuningAngle/9ft")),
            new Point2D.Double(10.0, LiveTuningHandler.getInstance().getValue("ShotTuningAngle/10ft")),
            new Point2D.Double(11.0, LiveTuningHandler.getInstance().getValue("ShotTuningAngle/11ft")),
            new Point2D.Double(12.0, LiveTuningHandler.getInstance().getValue("ShotTuningAngle/12ft")),
            new Point2D.Double(13.0, LiveTuningHandler.getInstance().getValue("ShotTuningAngle/13ft")),
            new Point2D.Double(14.0, LiveTuningHandler.getInstance().getValue("ShotTuningAngle/14ft")),
            new Point2D.Double(15.0, LiveTuningHandler.getInstance().getValue("ShotTuningAngle/15ft")),
            new Point2D.Double(16.0, LiveTuningHandler.getInstance().getValue("ShotTuningAngle/16ft")),
            new Point2D.Double(17.0, LiveTuningHandler.getInstance().getValue("ShotTuningAngle/17ft")),
            new Point2D.Double(18.0, LiveTuningHandler.getInstance().getValue("ShotTuningAngle/18ft")),
            new Point2D.Double(19.0, LiveTuningHandler.getInstance().getValue("ShotTuningAngle/19ft")),
            new Point2D.Double(20.0, LiveTuningHandler.getInstance().getValue("ShotTuningAngle/20ft")),
            new Point2D.Double(21.0, LiveTuningHandler.getInstance().getValue("ShotTuningAngle/21ft"))
        );
    }

    public static LinearInterpolationTable getLiveFlightTimeTable(){
        return new LinearInterpolationTable(
            new Point2D.Double(5.0, LiveTuningHandler.getInstance().getValue("ShotFlightTime/5ft")),
            new Point2D.Double(6.0, LiveTuningHandler.getInstance().getValue("ShotFlightTime/6ft")),
            new Point2D.Double(7.0, LiveTuningHandler.getInstance().getValue("ShotFlightTime/7ft")),
            new Point2D.Double(8.0, LiveTuningHandler.getInstance().getValue("ShotFlightTime/8ft")),
            new Point2D.Double(9.0, LiveTuningHandler.getInstance().getValue("ShotFlightTime/9ft")),
            new Point2D.Double(10.0, LiveTuningHandler.getInstance().getValue("ShotFlightTime/10ft")),
            new Point2D.Double(11.0, LiveTuningHandler.getInstance().getValue("ShotFlightTime/11ft")),
            new Point2D.Double(12.0, LiveTuningHandler.getInstance().getValue("ShotFlightTime/12ft")),
            new Point2D.Double(13.0, LiveTuningHandler.getInstance().getValue("ShotFlightTime/13ft")),
            new Point2D.Double(14.0, LiveTuningHandler.getInstance().getValue("ShotFlightTime/14ft")),
            new Point2D.Double(15.0, LiveTuningHandler.getInstance().getValue("ShotFlightTime/15ft")),
            new Point2D.Double(16.0, LiveTuningHandler.getInstance().getValue("ShotFlightTime/16ft")),
            new Point2D.Double(17.0, LiveTuningHandler.getInstance().getValue("ShotFlightTime/17ft")),
            new Point2D.Double(18.0, LiveTuningHandler.getInstance().getValue("ShotFlightTime/18ft")),
            new Point2D.Double(19.0, LiveTuningHandler.getInstance().getValue("ShotFlightTime/19ft")),
            new Point2D.Double(20.0, LiveTuningHandler.getInstance().getValue("ShotFlightTime/20ft")),
            new Point2D.Double(21.0, LiveTuningHandler.getInstance().getValue("ShotFlightTime/21ft"))
        );
    }
}
