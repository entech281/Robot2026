package frc.robot.sensors.TurretHomeSwitch;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.entech.sensors.EntechSensor;
import frc.robot.RobotConstants;

public class TurretHomeSwitch extends EntechSensor<TurretHomeOutput> {
    private static final boolean ENABLED = true;
    private DigitalInput sensor;

    @Override
    public String getName() {
        return "HallEffectSensor";
    }

    @Override
    public void initialize() {
        if (ENABLED) {
            sensor = new DigitalInput(RobotConstants.PORTS.DIO.TURRET_HOME_SWITCH);
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
    protected TurretHomeOutput toOutputs() {
        TurretHomeOutput output = new TurretHomeOutput();

        if (ENABLED) {
            output.setPressed(!sensor.get());
        } else {
            output.setPressed(false);
        }

        return output;
    }

}
