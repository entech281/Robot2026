package frc.robot.sensors.HomeTurretSwitch;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.entech.sensors.EntechSensor;
import frc.robot.RobotConstants;

public class HomeTurretSwitch extends EntechSensor<HomeTurretOutput> {
    private static final boolean ENABLED = true;
    private DigitalInput sensor;

    @Override
    public String getName() {
        return "HomeTurretSwitch";
    }

    @Override
    public void initialize() {
        if (ENABLED) {
            sensor = new DigitalInput(8);
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
    protected HomeTurretOutput toOutputs() {
        HomeTurretOutput output = new HomeTurretOutput();

        if (ENABLED) {
            output.setPressed(!sensor.get());
        } else {
            output.setPressed(false);
        }

        return output;
    }

}
