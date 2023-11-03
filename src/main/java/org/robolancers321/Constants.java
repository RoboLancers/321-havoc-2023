/* (C) Robolancers 2024 */
package org.robolancers321;

import org.robolancers321.subsystems.arm.InverseArmKinematics;

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
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kManipulatorControllerPort = 1;
    public static final double kJoystickDeadband = 0.1;
  }

  public static class Arm {
    public static class Anchor {
      public static final int kAnchorPort = 15;
      public static final boolean kMotorInverted = true;
      public static final boolean kEncoderInverted = true;
      public static final int kCurrentLimit = 50; // 40 - 60
      public static final double kGearRatio = 1;
      public static final double kNominalVoltage = 12.0;

      /*
      velocity from encoder is rotations/s
      rotations/s * (meters/rotations) = meters/s
      m/s what we need for motion profile

      RelativeEncoder - 360 deg/gearRatio
      Invert Motor for correct encoder values. setReference negative for correct output

      AbsoluteEncoder - 360
      Invert Encoder for correct readings. Change sign of setReference and invert Motor for correct ouputs
      */
      public static final double kdistancePerRotation = 360;

      public static final double kP = 0.01;
      public static final double kI = 0;
      public static final double kD = 0.00015;
      public static final int kPIDSlot = 0;

      public static final double kS = 0;
      public static final double kG = 0;
      public static final double kV = 0;
      public static final double kA = 0;
      public static ArmFeedforward ANCHOR_FEEDFORWARD = new ArmFeedforward(kS, kG, kV, kA);

      public static final double maxVelocity = 0;
      public static final double maxAcceleration = 0;
      public static TrapezoidProfile.Constraints ANCHOR_CONSTRAINTS =
          new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration);

      public static final double kMinAngle = 40;
      public static final double kMaxAngle = 120; // ! TODO: is this too far, might be damaging to the hard stop
      public static final boolean kEnableSoftLimit = false;
      public static final double kZeroPosition = 0;

      public static double kMaxOutput = 0.2;
      public static double kMinOutput = -0.05; //-0.1
      public static final double kAnchorLength = Units.inchesToMeters(34.5); // in
      public static final double kTolerance = 7.0;
    }

    public static class Floating {
      public static final int kFloatingPort = 16;
      public static final boolean kMotorInverted = false;
      public static final boolean kEncoderInverted = false;
      public static final int kCurrentLimit = 40; // 40 - 60
      public static final double kNominalVoltage = 12.0;
      public static final double kGearRatio = 25;
      public static final double kdistancePerRotation = 360;

      public static final double kP = 0.027;
      public static final double kI = 0;
      public static final double kD = 0.0001;
      public static final int kPIDSlot = 0;

      public static final double kS = 0;
      public static final double kG = 0;
      public static final double kV = 0;
      public static final double kA = 0;
      public static ArmFeedforward FLOATING_FEEDFORWARD = new ArmFeedforward(kS, kG, kV, kA);

      public static final double maxVelocity = 40;
      public static final double maxAcceleration = 60;
      public static TrapezoidProfile.Constraints FLOATING_CONSTRAINTS =
          new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration);

      public static final double kMinAngle = -55;
      public static final double kMaxAngle = 30;
      public static final boolean kEnableSoftLimit = false;
      public static final double kZeroPosition = 0;


      public static double kMaxOutput = 0.3; // 0.26; //0.26
      public static double kMinOutput = -0.15;
      public static final double kFloatingLength = 1.0 * Units.inchesToMeters(35); // in
      public static final double kTolerance = 7.0;
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
      kCANCoderConfig.sensorCoefficient = (2.0 * Math.PI) / (4096.0);
      kCANCoderConfig.unitString = "rad";
      kCANCoderConfig.absoluteSensorRange = AbsoluteSensorRange.Signed_PlusMinus180;
      kCANCoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
    }

    public static final double kTrackWidthMeters = Units.inchesToMeters(17.5);
    public static final double kWheelBaseMeters = Units.inchesToMeters(17.5);

    public static final double kWheelRadiusMeters = Units.inchesToMeters(1.5);
    public static final double kGearRatio = 6.8;

    public static final double kMaxSpeedMetersPerSecond = 
    4.0;
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

  public enum RawArmSetpoints {
    /* From game manual, y is from carpet, z is from front of grid
    SHELF - 37.375 in high + 13 in from cone = 50.375 in high 
    MID - 34 in high, 22.75 in
    HIGH - 46 in high, 39.75 in
    y offset = 17, z offset = 12

    SHELF(85, 1.63) - 50.375 in, 26 in 
    MID(82.86, -29.5) - 34 in, 22.75 in
    HIGH(60.5, 6.5) - 51 in, 39.75 in

    11 deg of error for anchor
     */

    // SHELFCONE(100, 15), 
    SHELFCONE(102, 7.4), // 102, 7.4
    SHELFCUBE(93.5, 5.5), // 93.5, 5.5
    MID(86, 346-360), // 86, 346
    HIGH(60.7, 19.7), // 60.7, 14.7
    CONTRACT(110, -31);

    public final double anchor;

    public final double floating;

    RawArmSetpoints(double anchor, double floating) {
      this.anchor = anchor;
      this.floating = floating;
    }
  }

  public enum ArmSetpoints {
    //Dont use this
    SHELF(50.375, 0),
    MID(34, 28.75),
    HIGH(46, 39.75),
    TEST(0, 0);

    private double anchor;
    private double floating;
    private double yOffset = 17; // in
    private double zOffset = 12;

    ArmSetpoints(double y, double z) {
      InverseArmKinematics.Output angles = InverseArmKinematics.calculate(y + this.yOffset, z);

      this.anchor = angles.anchor;
      this.floating = angles.anchor - angles.floating;
    }

    public double getAnchor() {
      return this.anchor;
    }

    public double getFloating() {
      return this.floating;
    }
  }

  public static class Intake {
    public static final int kPort = 17;
    public static final double kLowVelocity = 1000;
    public static final double kMaxVelocity = 10500;
  }

  public static class IntakePID {
    public static final double kP = 0.000;
    public static final double kI = 0.000;
    public static final double kD = 0.000;
    public static final double kFF = 0.0001;
  }
}
