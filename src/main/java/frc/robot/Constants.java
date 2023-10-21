/* (C) Robolancers 2024 */
package frc.robot;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final double kPeriodSeconds = Robot.kDefaultPeriod;

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kJoystickDeadband = 0.1;
  }

  public static class Arm {
    public static class Anchor {
      public static final int kAnchorPort = 0;
      public static final int kAnchorEncoderPort = 0;
      public static final boolean kInverted = false;
      public static final int kCurrentLimit = 60; // 40 - 60
      public static final double kGearRatio = 1;

      /*
      velocity from encoder is rotations/s
      rotations/s * (meters/rotations) = meters/s
      m/s what we need for motion profile

      */
      public static final double kdistancePerRotation = 0;

      public static final double kP = 0;
      public static final double kI = 0;
      public static final double kD = 0;

      public static final double ks = 0;
      public static final double kg = 0;
      public static final double kv = 0;
      public static final double ka = 0;
      public static final ArmFeedforward ANCHOR_FEEDFORWARD = new ArmFeedforward(ks, kg, kv, ka);

      public static final double maxVelocity = 0;
      public static final double maxAcceleration = 0;
      public static final TrapezoidProfile.Constraints ANCHOR_CONSTRAINTS =
          new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration);

      public static final double kMinAngle = Double.NEGATIVE_INFINITY;
      public static final double kMaxAngle = Double.POSITIVE_INFINITY;
      public static final boolean kEnableSoftLimit = false;
      public static final double kZeroPosition = 0;

      public static final double kMaxOutput = Double.POSITIVE_INFINITY;
      public static final double kMinOutput = Double.NEGATIVE_INFINITY;
    }

    public static class Floating {
      public static final int kFloatingPort = 0;
      public static final int kFloatingEncoderPort = 0;
      public static final boolean kInverted = false;
      public static final int kCurrentLimit = 60; // 40 - 60
      public static final double kGearRatio = 1;
      public static final double kdistancePerRotation = 0;

      public static final double kP = 0;
      public static final double kI = 0;
      public static final double kD = 0;

      public static final double ks = 0;
      public static final double kg = 0;
      public static final double kv = 0;
      public static final double ka = 0;
      public static final ArmFeedforward FLOATING_FEEDFORWARD = new ArmFeedforward(ks, kg, kv, ka);

      public static final double maxVelocity = 0;
      public static final double maxAcceleration = 0;
      public static final TrapezoidProfile.Constraints FLOATING_CONSTRAINTS =
          new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration);

      public static final double kMinAngle = Double.NEGATIVE_INFINITY;
      public static final double kMaxAngle = Double.POSITIVE_INFINITY;
      public static final boolean kEnableSoftLimit = false;
      public static final double kZeroPosition = 0;

      public static final double kMaxOutput = Double.POSITIVE_INFINITY;
      public static final double kMinOutput = Double.NEGATIVE_INFINITY;
    }
  }

  public static class Swerve {
    public static final class ModuleConfig {
      public final String id;
      public final int kDriveId;
      public final int kTurnId;
      public final int kTurnEncoderId;
      public final double magOffsetDeg;
      public final boolean driveIsInverted;
      public final boolean turnIsInverted;

      public ModuleConfig(
          final String idString,
          final int driveId,
          final int turnId,
          final int turnEncoderId,
          final double magOffsetDeg,
          final boolean driveIsInverted,
          final boolean turnIsInverted) {
        this.id = idString;
        this.kDriveId = driveId;
        this.kTurnId = turnId;
        this.kTurnEncoderId = turnEncoderId;
        this.magOffsetDeg = magOffsetDeg;
        this.driveIsInverted = driveIsInverted;
        this.turnIsInverted = turnIsInverted;
      }
    }

    public static final ModuleConfig frontLeft =
        new ModuleConfig("FrontLeft", 19, 18, 3, 58.87907200420464, true, false);
    public static final ModuleConfig frontRight =
        new ModuleConfig("FrontRight", 11, 10, 1, -128.44749934250595, true, false);
    public static final ModuleConfig backLeft =
        new ModuleConfig("BackLeft", 3, 2, 2, 107.41925934100429, true, false);
    public static final ModuleConfig backRight =
        new ModuleConfig("BackRight", 5, 6, 4, -142.90441434353835, true, false);

    public static final CANCoderConfiguration kCANCoderConfig = new CANCoderConfiguration();

    static {
      kCANCoderConfig.sensorCoefficient = (2.0 * Math.PI) / (4096.0); // 7.0 * (180.0) / ((2526.0 + 2967.0) * 150.0);
      kCANCoderConfig.unitString = "rad";
      kCANCoderConfig.absoluteSensorRange = AbsoluteSensorRange.Signed_PlusMinus180;
      kCANCoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
    }

    public static final double kTrackWidthMeters = Units.inchesToMeters(17.5);
    public static final double kWheelBaseMeters = Units.inchesToMeters(17.5);

    public static final double kWheelRadiusMeters = Units.inchesToMeters(1.5);
    public static final double kGearRatio = 6.8;

    public static final double kMaxSpeedMetersPerSecond = 4.0;
    public static final double kMaxOmegaRadiansPerSecond = 1.5 * Math.PI;

    public static final double kRPMToMetersPerSecond =
        2 * Math.PI * kWheelRadiusMeters / (kGearRatio * 60.0);

    public static final SwerveDriveKinematics kSwerveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(kTrackWidthMeters / 2, kWheelBaseMeters / 2), // front left
            new Translation2d(kTrackWidthMeters / 2, -kWheelBaseMeters / 2), // front right
            new Translation2d(-kTrackWidthMeters / 2, kWheelBaseMeters / 2), // back left
            new Translation2d(-kTrackWidthMeters / 2, -kWheelBaseMeters / 2) // back right
            );

    // TODO: tune coeffs.
    public static final class Drive {
      public static final double kP = 0.0;
      public static final double kI = 0.0;
      public static final double kD = 0.0;
      public static final double kFF = 0.30;
    }

    public static final class Turn {
      public static final double kP = 0.4;
      public static final double kI = 0.0;
      public static final double kD = 0.002;
      public static final double kFF = 0.0;
    }
  }
}
