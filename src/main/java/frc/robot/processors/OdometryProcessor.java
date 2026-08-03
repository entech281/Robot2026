package frc.robot.processors;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotConstants;
import frc.robot.io.RobotIO;
import frc.robot.sensors.vision.VisionOutput;
import frc.robot.sensors.vision.VisionPose;

import static edu.wpi.first.units.Units.Degrees;

import java.util.List;

public class OdometryProcessor {
  private SwerveDrivePoseEstimator estimator;
  private boolean integrateVision = true;
  private Field2d field = new Field2d();

  public Pose2d getEstimatedPose() {
    return estimator.getEstimatedPosition();
  }

  public void createEstimator() {
    estimator = new SwerveDrivePoseEstimator(RobotConstants.DrivetrainConstants.DRIVE_KINEMATICS,
        Rotation2d.fromDegrees(RobotIO.getInstance().getGyroOutput().getYaw().in(Degrees)),
        RobotIO.getInstance().getDriveOutput().getModulePositions(),
        RobotConstants.ODOMETRY.INITIAL_POSE);
    field.setRobotPose(getEstimatedPose());
    SmartDashboard.putData(field);
  }

  public void update() {
    double[] timestamps = RobotIO.getInstance().getDriveOutput().getTimestamps();

    for (int i = 0; i < timestamps.length; i++) {
      List<double[]> drivePositions = RobotIO.getInstance().getDriveOutput().getDrivePositions();
      List<double[]> turningPositions = RobotIO.getInstance().getDriveOutput().getTurningPositions();

      SwerveModulePosition[] positionsAtTime = { null, null, null, null };
      for (int j = 0; j < 4; j++) {
        positionsAtTime[j] = (new SwerveModulePosition(drivePositions.get(j)[i],
            new Rotation2d(turningPositions.get(j)[i])));
      }

      estimator.updateWithTime(timestamps[i],
          Rotation2d.fromDegrees(RobotIO.getInstance().getGyroOutput().getYaw().in(Degrees)),
          positionsAtTime);
    }

    if (integrateVision) {
      for (VisionPose vp : RobotIO.getInstance().getVisionOutput().getVisionPoses()) {
        addVisionEstimatedPose(vp, RobotIO.getInstance().getVisionOutput(),
            Rotation2d.fromDegrees(RobotIO.getInstance().getGyroOutput().getYaw().in(Degrees)));
      }
    }

    estimator.update(Rotation2d.fromDegrees(RobotIO.getInstance().getGyroOutput().getYaw().in(Degrees)),
        RobotIO.getInstance().getDriveOutput().getModulePositions());

    RobotIO.getInstance().updateOdometryPose(getEstimatedPose());
    field.setRobotPose(getEstimatedPose());
  }

  public void addVisionEstimatedPose(Pose2d visionPose, double timeStamp, Rotation2d yaw) {
    Pose2d fixedVisionPose = new Pose2d(visionPose.getTranslation(), yaw);
    estimator.addVisionMeasurement(fixedVisionPose, timeStamp);
  }

  public void addVisionEstimatedPose(VisionPose vp, VisionOutput vo, Rotation2d yaw) {
    Pose2d visionPose = vp.getPose2d();
    if (visionPose.getX() < -RobotConstants.ODOMETRY.FIELD_BORDER_MARGIN
        || visionPose.getX() > vo.getFieldLength() + RobotConstants.ODOMETRY.FIELD_BORDER_MARGIN
        || visionPose.getY() < -RobotConstants.ODOMETRY.FIELD_BORDER_MARGIN
        || visionPose.getY() > vo.getFieldWidth() + RobotConstants.ODOMETRY.FIELD_BORDER_MARGIN) {
      return;
    }

    double xyStdDev = RobotConstants.ODOMETRY.XY_STD_DEV_COEFFICIENT
        * Math.pow(vp.getAvgDistanceToTags(), 1.2)
        / Math.pow(vp.getTrackedTargets().size(), 2.0);

    Pose2d fixedVisionPose = new Pose2d(visionPose.getTranslation(), yaw);

    estimator.addVisionMeasurement(fixedVisionPose, vp.getTimeStamp(),
        VecBuilder.fill(xyStdDev, xyStdDev, Double.POSITIVE_INFINITY));
  }

  public double calculateDistanceFromTarget(Pose2d target) {
    double xDist = getEstimatedPose().getX() - target.getX();
    double yDist = getEstimatedPose().getY() - target.getY();

    return Math.sqrt(xDist * xDist + yDist * yDist);
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    estimator.resetPosition(Rotation2d.fromDegrees(RobotIO.getInstance().getGyroOutput().getYaw().in(Degrees)),
        RobotIO.getInstance().getDriveOutput().getModulePositions(), pose);
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose      The pose to which to set the odometry.
   * @param gyroAngle the latest gyro angle.
   */
  public void resetOdometry(Pose2d pose, Rotation2d gyroAngle) {
    estimator.resetPosition(gyroAngle, RobotIO.getInstance().getDriveOutput().getModulePositions(),
        pose);
  }

  public boolean isIntegratingVision() {
    return this.integrateVision;
  }

  public void setIntegrateVision(boolean integrateVision) {
    this.integrateVision = integrateVision;
  }
}
