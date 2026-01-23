package frc.robot.livetuning;

import java.util.Arrays;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
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
                        / (RobotConstants.SwerveModuleConstants.WHEEL_DIAMETER_METERS * Math.PI)
                        * Math.PI * 2,
                RobotIO.getInstance().getDriveOutput().getFrontRightDrive().getCurrentPosition()
                        / (RobotConstants.SwerveModuleConstants.WHEEL_DIAMETER_METERS * Math.PI)
                        * Math.PI * 2,
                RobotIO.getInstance().getDriveOutput().getRearLeftDrive().getCurrentPosition()
                        / (RobotConstants.SwerveModuleConstants.WHEEL_DIAMETER_METERS * Math.PI)
                        * Math.PI * 2,
                RobotIO.getInstance().getDriveOutput().getRearRightDrive().getCurrentPosition()
                        / (RobotConstants.SwerveModuleConstants.WHEEL_DIAMETER_METERS * Math.PI)
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
        gyroDelta += Math.abs(newAngle.getRadians() - lastAngle.getRadians());
        DriverStation.reportWarning("\n\n\n\nInit angle: " + lastAngle.getRadians()
                + "\nNew Angle: " + newAngle.getRadians() + "\n\nInit Positions: " + Arrays.toString(positions)
                + "\nNew Positions: "
                + Arrays.toString(newPositions) + "\n\nDelta Angle: " + new Rotation2d(gyroDelta).getRadians()
                + "\n Delta Wheels: " + wheelDelta,
                false);
        return (gyroDelta * RobotConstants.DrivetrainConstants.DRIVE_BASE_RADIUS_METERS) / wheelDelta;
    }
}