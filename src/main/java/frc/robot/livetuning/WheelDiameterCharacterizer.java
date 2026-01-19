package frc.robot.livetuning;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.RobotConstants;
import frc.robot.io.RobotIO;

public class WheelDiameterCharacterizer {
    private double[] positions;
    private Rotation2d lastAngle;
    private double gyroDelta;

    public WheelDiameterCharacterizer() {

    }

    public double[] getModulePositions() {
        return new double[] {
                RobotIO.getInstance().getDriveOutput().getFrontLeftDrive().getCurrentPosition()
                        / RobotConstants.SwerveModuleConstants.DRIVING_ENCODER_POSITION_FACTOR_METERS_PER_ROTATION
                        * Math.PI * 2,
                RobotIO.getInstance().getDriveOutput().getFrontRightDrive().getCurrentPosition()
                        / RobotConstants.SwerveModuleConstants.DRIVING_ENCODER_POSITION_FACTOR_METERS_PER_ROTATION
                        * Math.PI * 2,
                RobotIO.getInstance().getDriveOutput().getRearLeftDrive().getCurrentPosition()
                        / RobotConstants.SwerveModuleConstants.DRIVING_ENCODER_POSITION_FACTOR_METERS_PER_ROTATION
                        * Math.PI * 2,
                RobotIO.getInstance().getDriveOutput().getRearRightDrive().getCurrentPosition()
                        / RobotConstants.SwerveModuleConstants.DRIVING_ENCODER_POSITION_FACTOR_METERS_PER_ROTATION
                        * Math.PI * 2
        };
    }

    public Rotation2d getGyroAngle() {
        return Rotation2d.fromDegrees(RobotIO.getInstance().getNavXOutput().getYaw());
    }

    public void getInitialMeasurements() {
        positions = getModulePositions();
        lastAngle = getGyroAngle();
        gyroDelta = 0.0;
    }

    public double updateAndCalculate() {
        double[] newPositions = getModulePositions();
        Rotation2d newAngle = getGyroAngle();
        double wheelDelta = 0.0;
        for (int i = 0; i < 4; i++) {
            wheelDelta += Math.abs(newPositions[i] - positions[i]) / 4.0;
        }
        gyroDelta += Math.abs(newAngle.minus(lastAngle).getRadians());

        return (gyroDelta * RobotConstants.DrivetrainConstants.DRIVE_BASE_RADIUS_METERS) / wheelDelta;
    }
}