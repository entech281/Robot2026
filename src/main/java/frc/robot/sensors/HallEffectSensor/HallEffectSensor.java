package frc.robot.sensors.HallEffectSensor;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.entech.sensors.EntechSensor;
import frc.robot.RobotConstants;

public class HallEffectSensor extends EntechSensor<HallEffectOutput> {
    private static final boolean ENABLED = true;
    private DigitalInput sensor;

    @Override
    public String getName() {
        return "HallEffectSensor";
    }

    @Override
    public void initialize() {
        if (ENABLED) {
            sensor = new DigitalInput(RobotConstants.PORTS.DIO.HALL_EFFECT_SENSOR);
        }
    }

    @Override
    public boolean isEnabled() {
        return ENABLED;
    }

    @Override
    public Command getTestCommand() {
        return Commands.none();
    }

    @Override
    protected HallEffectOutput toOutputs() {
        HallEffectOutput output = new HallEffectOutput();

        if (ENABLED) {
            output.setMagnetDetected(!sensor.get());
        } else {
            output.setMagnetDetected(false);
        }

        return output;
    }

}
