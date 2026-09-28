package frc.robot.subsystems.transfer;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.drive.RobotDriveBase.MotorType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.entech.subsystems.EntechSubsystem;
import frc.entech.subsystems.SparkOutput;
import frc.robot.RobotConstants;
import frc.robot.io.RobotIO;




public class TransferSubsystem extends EntechSubsystem<TransferInput, TransferOutput> {
    private int loopCount = 0;
    private boolean motor1Connected = false;
    private boolean motor2Connected = false;
    private static final int HEALTH_CHECK_PERIOD_LOOPS = 10;

    private static final boolean ENABLED = true;
    private static final boolean BRAKING = false;

    private SparkMax transferMotor1;
    private SparkFlex transferMotor2;

    @Override
    public void initialize() {
        if (!ENABLED) {
            return;
        }

        try {
            transferMotor1 = new SparkMax(
                RobotConstants.PORTS.CAN.TRANSFER_MOTOR_1,
                MotorType.kBrushless
            );

            SparkMaxConfig motor1Config = new SparkMaxConfig();
            motor1Config
                .idleMode(BRAKING ? IdleMode.kBrake : IdleMode.kCoast)
                .smartCurrentLimit(30);

            transferMotor1.configure(
                motor1Config,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters
            );
        } catch (Exception exception) {
            transferMotor1 = null;
            DriverStation.reportWarning(
                "Transfer motor 1 failed to initialize: " + exception.getMessage(),
                false
            );
        }

        try {
            transferMotor2 = new SparkFlex(
                RobotConstants.PORTS.CAN.TRANSFER_MOTOR_2,
                MotorType.kBrushless
            );

            SparkFlexConfig motor2Config = new SparkFlexConfig();
            motor2Config
                .idleMode(BRAKING ? IdleMode.kBrake : IdleMode.kCoast)
                .smartCurrentLimit(30);

            transferMotor2.configure(
                motor2Config,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters
            );
        } catch (Exception exception) {
            transferMotor2 = null;
            DriverStation.reportWarning(
                "Transfer motor 2 failed to initialize: " + exception.getMessage(),
                false
            );
        }
    }

    @Override
    public boolean isEnabled() {
        return ENABLED;
    }

    @Override
    public void updateInputs(TransferInput input) {
        RobotIO.processInput(input);

        if (!ENABLED) {
            return;
        }

        // At 50 Hz, this checks CAN status every 0.2 seconds.
        if (loopCount++ % HEALTH_CHECK_PERIOD_LOOPS == 0) {
            motor1Connected = isMotorConnected(transferMotor1);
            motor2Connected = isMotorConnected(transferMotor2);
        }

        double speed = input.getSpeed();

        if (motor1Connected) {
            transferMotor1.set(-speed);
        }

        if (motor2Connected) {
            transferMotor2.set(-speed);
        }
}

    @Override
    public Command getTestCommand() {
        return new TestTransferCommand(this);
    }

    @Override
    protected TransferOutput toOutputs() {
        TransferOutput output = new TransferOutput();
        output.setBraking(BRAKING);

        if (transferMotor1 != null) {
            output.setTransferMotorOutput1(SparkOutput.createOutput(transferMotor1));
        }

        if (transferMotor2 != null) {
            output.setTransferMotorOutput2(SparkOutput.createOutput(transferMotor2));
        }

        return output;
    }

    private boolean isMotorConnected(SparkBase motor) {
        return motor != null && motor.getBusVoltage().isValid();
}
}