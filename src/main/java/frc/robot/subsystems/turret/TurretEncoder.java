package frc.robot.subsystems.turret;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.REVLibError;
import com.revrobotics.encoder.DetachedEncoder;

public class TurretEncoder extends DetachedEncoder {
    private final AbsoluteEncoder absoluteEncoder;
    private double velocityConversionFactor = 1.0;
    private double positionConversionFactor = 1.0;
    private double lastPostion = 0.0;
    private double relativePosition = 0.0;
    private double positionOffset = 0.0;
    public TurretEncoder(int id, Model model, AbsoluteEncoder absEncoder) {
        super(id, model);

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

        double deltaPositon = currentPosition - lastPostion;

        if (deltaPositon <= -0.5) {
            deltaPositon = deltaPositon + 1;
        } else if (deltaPositon > 0.5) {
            deltaPositon = deltaPositon - 1;
        }

        relativePosition = relativePosition + deltaPositon;

        lastPostion = currentPosition;

        return relativePosition;
    }

    @Override
    public double getAngle() {
        return getContinuousPosition() * positionConversionFactor;
    }

    @Override
    public double getPosition() {
        return getContinuousPosition() * positionConversionFactor;
    }

    @Override
    public double getRawAngle() {
        return absoluteEncoder.getPosition();
    }

    @Override
    public double getVelocity() {
        return absoluteEncoder.getVelocity() * velocityConversionFactor;
    }

    @Override
    public REVLibError setPosition(double position) {
        lastPostion = (position / positionConversionFactor) % 1;
        positionOffset = absoluteEncoder.getPosition() - lastPostion;
        relativePosition = position / positionConversionFactor;
        return REVLibError.kOk;
    }
}
