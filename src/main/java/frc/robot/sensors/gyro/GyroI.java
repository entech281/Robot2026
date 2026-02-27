package frc.robot.sensors.gyro;

import edu.wpi.first.units.measure.Angle;

public interface GyroI {
    void initialize();

    GyroOutput getOutput();

    void logUniqueData();

    void setAngleOffset(Angle angle);

    void zeroYaw();

    void reset();
}
