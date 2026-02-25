package frc.robot.sensors.gyro;

import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.wpilibj.ADIS16448_IMU;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ADIS16448 implements GyroI {
    private ADIS16448_IMU gyro;
    private Angle angleOffset = Angle.ofBaseUnits(0, Degrees);

    @Override
    public void initialize() {
        gyro = new ADIS16448_IMU();
        gyro.calibrate();
    }

    @Override
    public GyroOutput getOutput() {
        GyroOutput out = new GyroOutput();

        out.setAngleAdjustment(angleOffset);
        out.setYaw(Angle.ofBaseUnits(gyro.getAngle(), Degrees).minus(angleOffset));
        out.setYawRate(AngularVelocity.ofBaseUnits(gyro.getRate(), DegreesPerSecond));
        out.setTemperature(Temperature.ofBaseUnits(gyro.getTemperature(), Celsius));
        out.setChassisSpeeds(getChassisSpeeds());
        out.setRoll(Angle.ofBaseUnits(gyro.getGyroAngleY(), Degrees));
        out.setPitch(Angle.ofBaseUnits(gyro.getGyroAngleX(), Degrees));

        return out;
    }

    @Override
    public void logUniqueData() {
        Logger.recordOutput("ADIS16448Output/angleX", gyro.getGyroAngleX());
        Logger.recordOutput("ADIS16448Output/angleY", gyro.getGyroAngleY());
        Logger.recordOutput("ADIS16448Output/angleZ", gyro.getGyroAngleZ());
        Logger.recordOutput("ADIS16448Output/rateX", gyro.getGyroRateX());
        Logger.recordOutput("ADIS16448Output/rateY", gyro.getGyroRateY());
        Logger.recordOutput("ADIS16448Output/rateZ", gyro.getGyroRateZ());
        Logger.recordOutput("ADIS16448Output/accelX", gyro.getAccelX());
        Logger.recordOutput("ADIS16448Output/accelY", gyro.getAccelY());
        Logger.recordOutput("ADIS16448Output/accelZ", gyro.getAccelZ());
        Logger.recordOutput("ADIS16448Output/magX", gyro.getMagneticFieldX());
        Logger.recordOutput("ADIS16448Output/magY", gyro.getMagneticFieldY());
        Logger.recordOutput("ADIS16448Output/magZ", gyro.getMagneticFieldZ());
        Logger.recordOutput("ADIS16448Output/", gyro.getBarometricPressure());
        Logger.recordOutput("ADIS16448Output/", gyro.isConnected());
        SmartDashboard.putData(gyro);
    }

    @Override
    public void setAngleOffset(Angle angle) {
        angleOffset = angle;
    }

    @Override
    public void zeroYaw() {
        angleOffset = Angle.ofBaseUnits(gyro.getAngle(), Degrees);
    }

    private ChassisSpeeds getChassisSpeeds() {
        return new ChassisSpeeds();
    }

    @Override
    public void reset() {
        gyro.calibrate();
    }
}
