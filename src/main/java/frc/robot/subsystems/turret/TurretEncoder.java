package frc.robot.subsystems.turret;

import com.revrobotics.AbsoluteEncoder;

public class TurretEncoder {
    private final AbsoluteEncoder absoluteEncoder;
    private double velocityConversionFactor = 1.0;
    private double positionConversionFactor = 1.0;
    private double lastPosition = 0.0;
    private double relativePosition = 0.0;
    private double positionOffset = 0.0;

    public TurretEncoder(AbsoluteEncoder absEncoder) {
        absoluteEncoder = absEncoder;
    }

    public void setPositionConversionFactor(double factor) {
        positionConversionFactor = factor;
    }

    public void setVelocityConversionFactor(double factor) {
        velocityConversionFactor = factor;
    }

    private double getContinuousPosition() {
        double currentPosition = absoluteEncoder.getPosition() - positionOffset;

        double deltaPosition = currentPosition - lastPosition;

        if (deltaPosition <= -0.5) {
            deltaPosition = deltaPosition + 1;
        } else if (deltaPosition > 0.5) {
            deltaPosition = deltaPosition - 1;
        }

        relativePosition = relativePosition + deltaPosition;

        lastPosition = currentPosition;

        return relativePosition;
    }

    public double getPosition() {
        return getContinuousPosition() * positionConversionFactor;
    }

    public double getVelocity() {
        return absoluteEncoder.getVelocity() * velocityConversionFactor;
    }

    public void setPosition(double position) {
        lastPosition = (position / positionConversionFactor) % 1;
        positionOffset = absoluteEncoder.getPosition() - lastPosition;
        relativePosition = position / positionConversionFactor;
    }
}
