package frc.robot.subsystems.hood;

import com.revrobotics.jni.CANSparkJNI;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.drive.RobotDriveBase.MotorType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.entech.subsystems.EntechSubsystem;
import frc.entech.util.EntechUtils;
import frc.robot.RobotConstants;
import frc.robot.io.RobotIO;

public class HoodSubsystem extends EntechSubsystem<HoodInput, HoodOutput> {

    private static final boolean ENABLED = true;
    private static final boolean IS_INVERTED = false;

    private HoodInput currentInput = new HoodInput();

    private CANSparkMax hoodLeft;
    private CANSparkMax hoodRight;

    private IdleMode mode;

    public static double calculateMotorPositionFromDegrees(double degrees) {
      return degrees / RobotConstants.HOOD.HOOD_CONVERSION_FACTOR;
    }

    @Override
    public void initialize() {
      if (ENABLED) {
      // IMPORTANT! DO NOT BURN FLASH OR SET SETTINGS FOR THIS SUBSYSTEM in code!
      // we want to avoid accidently disabling the controller soft limits
      hoodLeft = new CANSparkJNI(RobotConstants.PORTS.CAN.HOOD_A, MotorType.kBrushless);
      hoodRight = new CANSparkJNI(RobotConstants.PORTS.CAN.HOOD_B, MotorType.kBrushless);
      hoodLeft.follow(hoodRight);
      hoodRight.getEncoder().setPosition(0.0);

      hoodLeft.setInverted(IS_INVERTED);
      hoodRight.setInverted(IS_INVERTED);
      hoodLeft.setIdleMode(IdleMode.kBrake);
      hoodRight.setIdleMode(IdleMode.kBrake);
      mode = IdleMode.kBrake;
      }
  }

    private double clampRequestedPosition(double position) {
      if (position < 0) {
        DriverStation.reportWarning("Hood tried to go to " + currentInput.getRequestedPosition()
            + " value was changed to " + RobotConstants.HOOD.LOWER_SOFT_LIMIT_DEG, false);
        return RobotConstants.HOOD.LOWER_SOFT_LIMIT_DEG;
      } else if (position > RobotConstants.HOOD.UPPER_SOFT_LIMIT_DEG) {
        DriverStation.reportWarning("Hood tried to go to " + currentInput.getRequestedPosition()
            + " value was changed to " + RobotConstants.HOOD.UPPER_SOFT_LIMIT_DEG, false);
        return RobotConstants.HOOD.UPPER_SOFT_LIMIT_DEG;
      } else {
        return position;
      }
    }
    @Override
    public boolean isEnabled() {
        return ENABLED;
    }

    @Override
    public void updateInputs(HoodInput input) {
        RobotIO.processInput(input);
        this.currentInput = input;
    }

    @Override
    public Command getTestCommand() {
        return new TestHoodCommand(this);
    }

    @Override
    protected HoodOutput toOutputs() {
        HoodOutput hoodOutput = new HoodOutput();
        hoodOutput.setMoving(hoodLeft.getEncoder().getVelocity() != 0);
        hoodOutput.setLeftBrakeModeEnabled(IdleMode.kBrake == mode);
        hoodOutput.setRightBrakeModeEnabled(IdleMode.kBrake == mode);
        hoodOutput.setCurrentPosition(
            hoodLeft.getEncoder().getPosition() * RobotConstants.HOOD.HOOD_CONVERSION_FACTOR);
        hoodOutput.setAtRequestedPosition(EntechUtils.isWithinTolerance(2,
            hoodOutput.getCurrentPosition(), currentInput.getRequestedPosition()));
        hoodOutput.setAtLowerLimit(
            hoodLeft.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen).isPressed());
        hoodOutput.setRequestedPosition(currentInput.getRequestedPosition());
        return hoodOutput;
    }
    
}
