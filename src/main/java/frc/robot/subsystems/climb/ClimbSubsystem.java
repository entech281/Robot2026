package frc.robot.subsystems.climb;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import frc.entech.subsystems.EntechSubsystem;
import frc.robot.RobotConstants;
// import frc.robot.commands.test.TestClimbCommand;
import frc.robot.io.RobotIO;

public class ClimbSubsystem extends EntechSubsystem<ClimbInput, ClimbOutput> {
  private static final boolean ENABLED = true;

  private ClimbInput currentInput = new ClimbInput();

  private SparkMax climbMotorRight;
  private SparkMax climbMotorLeft;

  private IdleMode mode;

@Override
public void initialize() {
    if (ENABLED) {
        climbMotorRight = new SparkMax(RobotConstants.PORTS.CAN.CLIMB_A, MotorType.kBrushless);
        climbMotorLeft = new SparkMax(RobotConstants.PORTS.CAN.CLIMB_B, MotorType.kBrushless);

        SparkMaxConfig configRight = new SparkMaxConfig();
        SparkMaxConfig configLeft = new SparkMaxConfig();

        configRight.inverted(false)
            .encoder.positionConversionFactor(RobotConstants.CLIMB.CLIMB_CONVERSION_FACTOR);
        configRight.idleMode(IdleMode.kCoast);
        setUpPIDConstants(configRight);

        configLeft.inverted(true)
            .encoder.positionConversionFactor(RobotConstants.CLIMB.CLIMB_CONVERSION_FACTOR);
        configLeft.idleMode(IdleMode.kCoast);
        setUpPIDConstants(configLeft);

        climbMotorRight.configure(configRight, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        climbMotorLeft.configure(configLeft, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        mode = IdleMode.kCoast;

        climbMotorRight.getEncoder().setPosition(0.0);
        climbMotorLeft.getEncoder().setPosition(0.0);
    }
}


private void setUpPIDConstants(SparkMaxConfig config) {
    config.closedLoop.pid(
        RobotConstants.CLIMB.KP, 
        RobotConstants.CLIMB.KI, 
        RobotConstants.CLIMB.KD
    );
}

  @Override
  public void periodic() {
    if (ENABLED) {
      if (currentInput.getActivate()) {
        if (currentInput.getFreeze()) {
          climbMotorRight.set(0.0);
          climbMotorLeft.set(0.0);

        } else {
          climbMotorRight.set(currentInput.getSpeedRight());
          climbMotorLeft.set(currentInput.getSpeedLeft());
        }
      } else {
        climbMotorRight.set(0.0);
        climbMotorLeft.set(0.0);
      }
    }
  }

  @Override
  public boolean isEnabled() {
    return ENABLED;
  }

  @Override
  public void updateInputs(ClimbInput input) {
    RobotIO.processInput(input);
    this.currentInput = input;
  }

  @Override
  public ClimbOutput toOutputs() {
    ClimbOutput climbOutput = new ClimbOutput();
    climbOutput.setActive(climbMotorRight.getEncoder().getVelocity() != 0);
    climbOutput.setBrakeModeEnabled(IdleMode.kBrake == mode);
    climbOutput.setCurrentPosition(climbMotorRight.getEncoder().getPosition());
    climbOutput.setExtended(climbMotorRight.getEncoder().getPosition() > 0);
    return climbOutput;
  }

  @Override
  public Command getTestCommand() {
    return null;
    // return new TestClimbCommand(this);
  }

  public void setPosition(double position) {
    climbMotorRight.getEncoder().setPosition(position);
    climbMotorLeft.getEncoder().setPosition(position);
  }
}