package frc.robot.sensors.gyro;

import static edu.wpi.first.units.Units.Degrees;

import org.littletonrobotics.junction.Logger;

import com.studica.frc.Navx;

import edu.wpi.first.units.measure.Angle;

public class NavX3 implements GyroI {
    private Angle angleOffset = Angle.ofRelativeUnits(0.0, Degrees);
    private final Navx gyro;

    public NavX3(int port) {
        gyro = new Navx(port, 50);
    }

    @Override
    public void initialize() {
    }

    @Override
    public GyroOutput getOutput() {
        GyroOutput out = new GyroOutput();

        out.setPitch(gyro.getPitch());
        out.setRoll(gyro.getRoll());
        out.setYaw(gyro.getYaw().minus(angleOffset));
        out.setYawRate(gyro.getAngularVel()[2]);
        out.setTemperature(gyro.getTemperature());
        out.setAngleAdjustment(angleOffset);

        return out;
    }

    @Override
    public void logUniqueData() {
        Logger.recordOutput("NavX3Output/accelX", gyro.getLinearAccel()[0]);
        Logger.recordOutput("NavX3Output/accelY", gyro.getLinearAccel()[1]);
        Logger.recordOutput("NavX3Output/accelZ", gyro.getLinearAccel()[2]);
        Logger.recordOutput("NavX3Output/quaternion", gyro.getQuat9D());
        Logger.recordOutput("NavX3Output/angularVelX", gyro.getAngularVel()[1]);
        Logger.recordOutput("NavX3Output/angularVelY", gyro.getAngularVel()[1]);
        Logger.recordOutput("NavX3Output/angularVelZ", gyro.getAngularVel()[2]);
    }

    @Override
    public void setAngleOffset(Angle angle) {
        angleOffset = angle;
    }

    @Override
    public void zeroYaw() {
        angleOffset = gyro.getYaw();
    }

    @Override
    public void reset() {
        gyro.resetYaw();
    }

}
