package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;

import java.util.Map;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.robot.livetuning.LiveTuningHandler;
import frc.robot.sensors.gyro.GyroSensor.GyroHardware;
import frc.robot.util.ShooterCalculator;
import frc.robot.util.ShooterCalculator.ShotDataRange.ShotData;

public final class RobotConstants {
  public static final GyroHardware GYRO_HARDWARE = GyroHardware.NAVX3;
  public static final double TIME_PER_PERIODICAL_LOOP_SECONDS = 0.00;

  public static interface AccelerationFilter {
    public static final double DIRECTION_SLEW_RATE = 0.95; // radians per second
    public static final double MAGNITUDE_SLEW_RATE = 1.0;
    // 2.0; //1.8; // percent per second (1 = 100%)
    public static final double ROTATIONAL_SLEW_RATE = 3.5;
  }

  public static interface DrivetrainConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double MAX_SPEED_METERS_PER_SECOND = 6.0; // 4.42; //4.8;
    public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = 4 * Math.PI;
    // radians per second

    public static final double DIRECTION_SLEW_RATE = 3; // radians per second
    public static final double MAGNITUDE_SLEW_RATE = 10;
    // 2.0; //1.8; // percent per second (1 = 100%)
    public static final double ROTATIONAL_SLEW_RATE = 6;
    // 20.0; //2.0; // percent per second (1 = 100%)

    // Chassis configuration
    public static final double TRACK_WIDTH_METERS = Units.inchesToMeters(20.75);

    // Distance between centers of right and left wheels on robot
    public static final double WHEEL_BASE_METERS = Units.inchesToMeters(22.75);

    // Distance to farthest module
    public static final double DRIVE_BASE_RADIUS_METERS = Math.sqrt(Math.pow(WHEEL_BASE_METERS / 2.0, 2)
        + Math.pow(TRACK_WIDTH_METERS / 2.0, 2));

    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(
        new Translation2d(WHEEL_BASE_METERS / 2, TRACK_WIDTH_METERS / 2),
        new Translation2d(WHEEL_BASE_METERS / 2, -TRACK_WIDTH_METERS / 2),
        new Translation2d(-WHEEL_BASE_METERS / 2, TRACK_WIDTH_METERS / 2),
        new Translation2d(-WHEEL_BASE_METERS / 2, -TRACK_WIDTH_METERS / 2));

    public static final boolean GYRO_REVERSED = false;
    public static final boolean RATE_LIMITING = false;
  }

  public static interface SwerveModuleConstants {
    public static final double FREE_SPEED_RPM = 6784;

    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth
    // will result in a
    // robot that drives faster).
    public static final int DRIVING_MOTOR_PINION_TEETH = 14;

    // Invert the turning encoder, since the output shaft rotates in the opposite
    // direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean TURNING_ENCODER_INVERTED = true;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double DRIVING_MOTOR_FREE_SPEED_RPS = FREE_SPEED_RPM / 60;
    public static final double WHEEL_DIAMETER_METERS = 0.09802701871182373;
    public static final double WHEEL_CIRCUMFERENCE_METERS = WHEEL_DIAMETER_METERS * Math.PI;
    public static final double DRIVING_MOTOR_REDUCTION = (45.0 * 17 * 50) / (DRIVING_MOTOR_PINION_TEETH * 15 * 27);
    public static final double DRIVE_WHEEL_FREE_SPEED_RPS = (DRIVING_MOTOR_FREE_SPEED_RPS * WHEEL_CIRCUMFERENCE_METERS)
        / DRIVING_MOTOR_REDUCTION;

    public static final double DRIVING_ENCODER_POSITION_FACTOR_METERS_PER_ROTATION = (WHEEL_DIAMETER_METERS * Math.PI)
        / DRIVING_MOTOR_REDUCTION; // meters, per rotation
    public static final double DRIVING_ENCODER_VELOCITY_FACTOR_METERS_PER_SECOND_PER_RPM = ((WHEEL_DIAMETER_METERS
        * Math.PI) / DRIVING_MOTOR_REDUCTION) / 60.0;
    // meters per second, per RPM

    public static final double TURNING_MOTOR_REDUCTION = 150.0 / 7.0;
    // ratio between internal relative encoder and
    // Through Bore (or Thrifty in our case)
    // absolute encoder - 150.0 / 7.0

    public static final double TURNING_ENCODER_POSITION_FACTOR_RADIANS_PER_ROTATION = (2 * Math.PI)
        / TURNING_MOTOR_REDUCTION; // radians, per rotation
    public static final double TURNING_ENCODER_VELOCITY_FACTOR_RADIANS_PER_SECOND_PER_RPM = (2 * Math.PI)
        / TURNING_MOTOR_REDUCTION / 60.0; // radians per second, per RPM

    public static final double TURNING_ENCODER_POSITION_PID_MIN_INPUT_RADIANS = 0; // radians
    public static final double TURNING_ENCODER_POSITION_PID_MAX_INPUT_RADIANS = (2 * Math.PI);
    // radians

    public static final double DRIVING_P = 0.5; // Origional P = 0.07
    public static final double DRIVING_I = 0;
    public static final double DRIVING_D = 0;
    public static final double DRIVING_FF = 1 / DRIVE_WHEEL_FREE_SPEED_RPS;
    public static final double DRIVING_MIN_OUTPUT_NORMALIZED = -1;
    public static final double DRIVING_MAX_OUTPUT_NORMALIZED = 1;

    public static final double TURNING_P = 1.5;
    // 1.0; // 1.0 might be a bit too much - reduce a bit if needed
    public static final double TURNING_I = 0;
    public static final double TURNING_D = 0;
    public static final double TURNING_FF = 0;
    public static final double TURNING_MIN_OUTPUT_NORMALIZED = -1;
    public static final double TURNING_MAX_OUTPUT_NORMALIZED = 1;

    public static final IdleMode DRIVING_MOTOR_IDLE_MODE = IdleMode.kBrake;
    public static final IdleMode TURNING_MOTOR_IDLE_MODE = IdleMode.kBrake;

    public static final int DRIVING_MOTOR_CURRENT_LIMIT_AMPS = 160; // 50; // amps
    public static final int TURNING_MOTOR_CURRENT_LIMIT_AMPS = 40; // amps

    public static final double FRONT_LEFT_VIRTUAL_OFFSET_RADIANS = 2.285;
    public static final double FRONT_RIGHT_VIRTUAL_OFFSET_RADIANS = 0.91;
    public static final double REAR_LEFT_VIRTUAL_OFFSET_RADIANS = 2.71;
    public static final double REAR_RIGHT_VIRTUAL_OFFSET_RADIANS = -2.9;
  }

  public static interface LiveTuning {
    public static final Map<String, Double> VALUES = Map.ofEntries(
        Map.entry("ShooterSubsystem/SetSpeed", 0.0),
        Map.entry("IntakeSubsystem/SetSpeed", 0.0),
        Map.entry("TransferSubsystem/SetSpeed", 0.0),
        Map.entry("TurretSubsystem/LowerLimitDegrees", -45.0),
        Map.entry("TurretSubsystem/UpperLimitDegrees", 45.0),
        Map.entry("TurretSubsystem/SofterLowerLimitDegrees", -40.0),
        Map.entry("TurretSubsystem/SofterUpperLimitDegrees", 45.0),
        Map.entry("TurretSubsystem/PresetOneDegrees", 0.0),
        Map.entry("TurretSubsystem/PresetTwoDegrees", 0.0),
        Map.entry("HoodSubsystem/PresetOneDegrees", 35.0),
        Map.entry("HoodSubsystem/PresetTwoDegrees", 35.0),
        Map.entry("ShooterSubsystem/PresetOneRPM", RobotConstants.SHOOTER.MAX_RPM),
        Map.entry("ShooterSubsystem/PresetTwoRPM", RobotConstants.SHOOTER.MAX_RPM),
        Map.entry("ShiftStateTracker/WarningSeconds", 5.0),
        Map.entry("TurretSubsystem/LiveAngle", 0.0));
  }

  public static interface PORTS {

    public static interface ANALOG {
      public static final int FRONT_LEFT_TURNING_ABSOLUTE_ENCODER = 0;
      public static final int REAR_LEFT_TURNING_ABSOLUTE_ENCODER = 2;
      public static final int FRONT_RIGHT_TURNING_ABSOLUTE_ENCODER = 1;
      public static final int REAR_RIGHT_TURNING_ABSOLUTE_ENCODER = 3;
    }

    public static interface CAN {
      public static final int FRONT_LEFT_DRIVING = 12;
      public static final int FRONT_RIGHT_DRIVING = 22;
      public static final int REAR_LEFT_DRIVING = 32;
      public static final int REAR_RIGHT_DRIVING = 42;

      public static final int FRONT_LEFT_TURNING = 11;
      public static final int FRONT_RIGHT_TURNING = 21;
      public static final int REAR_LEFT_TURNING = 31;
      public static final int REAR_RIGHT_TURNING = 41;

      public static final int SHOOTER_MOTOR_A = 55;
      public static final int SHOOTER_MOTOR_B = 56;

      public static final int INTAKE_MOTOR = 54;

      public static final int POWER_DISTRIBUTION_HUB = 1;
      public static final int TURRET_MOTOR = 57;
      public static final int HOOD_MOTOR = 52;
      public static final int HOPPER_MOTOR = 60;
      public static final int TRANSFER_MOTOR = 58;
      public static final int CLIMB_MOTOR = 59;
    }

    public static interface CONTROLLER {
      public static final double JOYSTICK_AXIS_THRESHOLD = 0.2;
      public static final int DRIVER_CONTROLLER = 0;
      public static final int SCORE_PANEL = 2;
      public static final int ALIGN_PANEL = 4;
      public static final int TEST_JOYSTICK = 2;
      public static final int TUNING_CONTROLLER = 3;
      public static final int SHIFT_LIGHT_OUTPUT = 5;

      public static interface BUTTONS_JOYSTICK {
        public static final int TWIST = 1;
        public static final int RUN_TESTS = 7;
        public static final int GYRO_RESET = 11;
        public static final int RESET_ODOMETRY = 3;
      }

      public static interface BUTTONS_XBOX {
        public static final int GYRO_RESET = 7;
        public static final int DRIVE_X = 3;
        public static final int RESET_ODOMETRY = 8;
        public static final int B = 2;
      }

    }

    public static interface DIO {
      public static final int HALL_EFFECT_SENSOR = 0;
    }
  }

  public interface SCORE_OPERATOR_PANEL {
    public static interface BUTTONS {
      // TODO: make real
      public static final int FIRE = 2;
      // TODO: make real
      public static final int AUTO_FIRE = 14;
      public static final int INTAKE = 6;
      public static final int OUTTAKE = 7;
      public static final int DEPLOY_HOPPER = 5;
      public static final int PRESET_1_FIRE = 2;
      public static final int PRESET_2_FIRE = 3;
      public static final int WON_AUTO_SWITCH = 11;
      public static final int SNOWBLOW_FIRE = 13;
    }

    public static interface SWITCHES {
    }
  }

  public static interface Vision {
    public static final Matrix<N3, N1> VISION_STD_DEVS = VecBuilder.fill(5, 5, 1000000);

    public static interface Cameras {
      public static final String CAMERA_A = "Arducam_Alpha";
      public static final String CAMERA_B = "Arducam_Beta";
      public static final String CAMERA_C = "Arducam_Charlie";
      public static final String CAMERA_D = "Arducam_Delta";
    }

    public static interface Filters {
      public static final double MAX_AMBIGUITY = 0.3; // Lower = more strict (0.2-0.3 is good)
      public static final double MAX_DISTANCE = 5.0; // Max distance to trust tags (meters)
      public static final int[] ALLOWED_TAGS = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16 }; // Update with
                                                                                                          // actual 2026
                                                                                                          // tag IDs

    }

    public static interface Resolution {
      public static final double[] COLOR_RESOLUTION = { 320, 240 };
    }

    public static interface Transforms {
      // Define where your camera is mounted on the robot/rig
      // *** YOU MUST MEASURE AND UPDATE THESE VALUES ***
      // Example values shown below - camera 0.3m forward, centered, 0.4m high, tilted
      // down 25 degrees
      Transform3d robotToCameraA = new Transform3d(
          new Translation3d(
              Units.inchesToMeters(-11.125), // X: meters forward from robot center (positive = forward)
              Units.inchesToMeters(11.875), // Y: meters left from robot center (positive = left)
              Units.inchesToMeters(8.25) // Z: meters up from ground (camera height)
          ),
          new Rotation3d(
              Math.toRadians(0), // Roll (rotation around X axis)
              Math.toRadians(0), // Pitch (rotation around Y axis, negative = tilted down)
              Math.toRadians(135) // Yaw (rotation around Z axis)
          ));

      Transform3d robotToCameraB = new Transform3d(
          new Translation3d(
              Units.inchesToMeters(-11.125), // X: meters forward from robot center (positive = forward)
              Units.inchesToMeters(-11.875), // Y: meters left from robot center (positive = left)
              Units.inchesToMeters(8.25) // Z: meters up from ground (camera height)
          ),
          new Rotation3d(
              Math.toRadians(0), // Roll (rotation around X axis)
              Math.toRadians(0), // Pitch (rotation around Y axis, negative = tilted down)
              Math.toRadians(-135) // Yaw (rotation around Z axis)
          ));

      Transform3d robotToCameraC = new Transform3d(
          new Translation3d(
              Units.inchesToMeters(-1.375), // X: meters forward from robot center (positive = forward)
              Units.inchesToMeters(10.375), // Y: meters left from robot center (positive = left)
              Units.inchesToMeters(16.25) // Z: meters up from ground (camera height)
          ),
          new Rotation3d(
              Math.toRadians(0), // Roll (rotation around X axis)
              Math.toRadians(0), // Pitch (rotation around Y axis, negative = tilted down)
              Math.toRadians(45) // Yaw (rotation around Z axis)
          ));

      Transform3d robotToCameraD = new Transform3d(
          new Translation3d(
              Units.inchesToMeters(-1.375), // X: meters forward from robot center (positive = forward)
              Units.inchesToMeters(-10.375), // Y: meters left from robot center (positive = left)
              Units.inchesToMeters(16.25) // Z: meters up from ground (camera height)
          ),
          new Rotation3d(
              Math.toRadians(0), // Roll (rotation around X axis)
              Math.toRadians(0), // Pitch (rotation around Y axis, negative = tilted down)
              Math.toRadians(-45) // Yaw (rotation around Z axis)
          ));
    }
  }

  public static interface AUTONOMOUS {
    public static final double MAX_MODULE_SPEED_METERS_PER_SECOND = 4.5; // 4.42

    public static final double TRANSLATION_CONTROLLER_P = 5;
    public static final double ROTATION_CONTROLLER_P = 5;
  }

  public static interface INDICATOR_VALUES {
    public static final double POSITION_UNKNOWN = -1.0;
    public static final double POSITION_NOT_SET = -1.1;
  }

  public static interface ODOMETRY {
    public static final int ODOMETRY_FREQUENCY = 150;
    public static final double FIELD_LENGTH_INCHES = 54 * 12 + 3.25;
    public static final double FIELD_WIDTH_INCHES = 26 * 12 + 11.25;

    public static final Translation2d INITIAL_TRANSLATION = new Translation2d(
        Units.inchesToMeters(FIELD_LENGTH_INCHES / 2),
        Units.inchesToMeters(FIELD_WIDTH_INCHES / 2));
    public static final Rotation2d INITIAL_ROTATION = Rotation2d.fromDegrees(0);

    public static final Pose2d INITIAL_POSE = new Pose2d(INITIAL_TRANSLATION, INITIAL_ROTATION);
  }

  public static interface OperatorMessages {
    public static final String SUBSYSTEM_TEST = "SubsystemTest";
  }

  public static interface TEST_CONSTANTS {
    public static final double STANDARD_TEST_LENGTH = 1;
  }

  public static interface TURRET {
    // TODO: Make these real
    public static final double INITIAL_POSITION_DEGREES = 0.0;
    // Turret closed-loop settings
    public static final double POSITION_CONVERSION_FACTOR_DEGREES = 3.2;// ;//207360 // encoder units -> degrees (set
                                                                        // appropriately)
    public static final double TURRET_POSITION_P = 0.02;
    public static final double TURRET_POSITION_I = 0.0;
    public static final double TURRET_POSITION_D = 0.0;
    public static final double TURRET_POSITION_FF = 0.0;
    public static final double TURRET_CRUISE_VELOCITY_RPM = 100.0; // max velocity for motion magic
    public static final double TURRET_MAX_ACCELERATION_RPM_PER_SECOND = 100.0; // max acceleration for motion magic
    public static final double TURRET_ALLOWED_PROFILE_ERROR_ROTATIONS = 0.5; // allowable error for motion magic
    public static final double TURRET_POSITION_TOLERANCE_DEGREES = 1.0; // considered at setpoint within this
    public static final double HOME_POSITION_DEGREES = 0.0; // position to reset to
    // preset manual positions (buttons will command these)
    public static final double TURRET_POSITION_PRESET_A_DEGREES = 0.0;
    public static final double TURRET_POSITION_PRESET_B_DEGREES = -20;
    public static final double TURRET_POSITION_PRESET_Y_DEGREES = 20;
    // small adjustment step used by any incremental commands
    public static final double TURRET_ADJUST_STEP_DEGREES = 5.0;

    public static final Pose3d BLUE_HUB_LOCATION = new Pose3d(Inches.of(182.11).in(Meters),
        Inches.of(158.845).in(Meters), Inches.of(0).in(Meters), new Rotation3d());
    public static final Pose3d RED_HUB_LOCATION = new Pose3d(Inches.of(469.11).in(Meters),
        Inches.of(158.845).in(Meters), Inches.of(0).in(Meters), new Rotation3d());

    public static final Pose3d BLUE_SNOWBLOW_TARGET = new Pose3d(0.0,
        Inches.of(158.845).in(Meters), Inches.of(0).in(Meters), new Rotation3d());

    public static final Pose3d RED_SNOWBLOW_TARGET = new Pose3d(Inches.of(651).in(Meters),
        Inches.of(158.845).in(Meters), Inches.of(0).in(Meters), new Rotation3d());

    public static final Translation2d TURRET_OFFSET = new Translation2d(-DrivetrainConstants.WHEEL_BASE_METERS / 2.0,
        0.0);
  }

  public static interface HOOD {
    // TODO: make these real
    public static final double POSITION_CONVERSION_FACTOR_DEGREES = 1.65441176471;
    public static final double HOOD_P = 3;
    public static final double HOOD_I = 0.0;
    public static final double HOOD_D = 0.0;
    public static final double INITIAL_POSITION_DEGREES = 0.0;
    public static final double HOOD_LOWER_LIMIT_DEGREES = 0.0;
    public static final double HOOD_UPPER_LIMIT_DEGREES = 25.0;
    public static final double HOOD_POSITION_TOLERANCE_DEGREES = 0.1;
    public static final double HOOD_CRUISE_VELOCITY_RPM = 2000.0;
    public static final double HOOD_MAX_ACCELERATION_RPM_PER_SECOND = 10000.0;
    public static final double HOOD_ALLOWED_PROFILE_ERROR_ROTATIONS = 0.5;
  }

  public static interface SHOOTER {
    public static final Transform3d SHOT_TRANSFORM = new Transform3d(0, 0, 0, new Rotation3d());
    public static final double WHEEL_RADIUS_METERS = 0.048229115; // TODO: Idk my ai made this number
    public static final double MAX_RPM = 6000.0;
    public static final ShotData SHOT_PRESET_ONE = new ShooterCalculator().new ShotDataRange().new ShotData(
        Degrees.of(LiveTuningHandler.getInstance().getValue("HoodSubsystem/PresetOneDegrees")),
        RPM.of(LiveTuningHandler.getInstance().getValue("ShooterSubsystem/PresetOneRPM")),
        Meters.of(WHEEL_RADIUS_METERS));
    public static final ShotData SHOT_PRESET_TWO = new ShooterCalculator().new ShotDataRange().new ShotData(
        Degrees.of(LiveTuningHandler.getInstance().getValue("HoodSubsystem/PresetTwoDegrees")),
        RPM.of(LiveTuningHandler.getInstance().getValue("ShooterSubsystem/PresetTwoRPM")),
        Meters.of(WHEEL_RADIUS_METERS));
  }

  public static interface HOPPER {
    public static final double DEPLOY_SPEED = -0.3; // Negative = downward, tune as needed
  }

  private RobotConstants() {
  }
}