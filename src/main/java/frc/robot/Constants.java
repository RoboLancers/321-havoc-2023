/* (C) Robolancers 2024 */
package frc.robot;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

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
  }

  public static final class Arm {
    public static final class Anchor {
      public static final int kAnchorPort = 22;

      public static final boolean kInverted = true;
      public static final double kZeroPosition =  16.0;
      public static final double kMinAngle =  16.0;
      public static final double kMaxAngle =  16.0;
      public static final boolean kEnableSoftLimit = true;
      public static final double kMaxOutput = 0.5; // going up
      public static final double kMinOutput = -0.4; // going down
      public static final int kCurrentLimit = 50; // 40 to 60

      public static final class PID {
        public static final double kP = 0;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final int kSlot = 0;
      }

      public static final class FF {
        public static final double ks = 0;
        public static final double kg = 0; // gravity FF most likely only tune this gain
        public static final double kv = 0;
        public static final double ka = 0;
        public static final ArmFeedforward ANCHOR_FEEDFORWARD = new ArmFeedforward(ks, kg, kv, ka);
      }

      public static final class MP {
        public static final double maxVel = 2.0;
        public static final double maxAccel = 1.0;
        public static final TrapezoidProfile.Constraints ANCHOR_CONSTRAINTS =
            new TrapezoidProfile.Constraints(maxVel, maxAccel);
      }

      public static final class Conversions {
        public static final double kRatio = (90.0 - 13.0) / (27.0);;
      }

      public static enum Setpoints {
        TEST_ANCHOR_SETPOINT(1, 0);

        public double position;
        public double velocity;

        Setpoints(double position, double velocity) {
          this.position = position;
          this.velocity = velocity;
        }
      }
    }

    public static final class Floating {
      public static final int kFloatingPort = 23;

      public static final boolean kInverted = true;
      public static final double kZeroPosition = 0;
      public static final double kMinAngle = 22.0;
      public static final double kMaxAngle = 180.0;
      public static final boolean kEnableSoftLimit = true;
      public static final double kMaxOutput = 0.5; // going up
      public static final double kMinOutput = -0.5; // going down
      public static final int kCurrentLimit = 50; // 40 to 60

      public static final class PID {
        public static final double kP = 0;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final int kSlot = 2;
      }

      public static final class FF {
        public static final double ks = 0;
        public static final double kg = 0; // gravity FF most likely only tune this gain
        public static final double kv = 0;
        public static final double ka = 0;
        public static final ArmFeedforward FLOATING_FEEDFORWARD =
            new ArmFeedforward(ks, kg, kv, ka);
      }

      public static final class MP {
        public static final double maxVel = 1.0;
        public static final double maxAccel = 1.0;
        public static final TrapezoidProfile.Constraints FLOATING_CONSTRAINTS =
            new TrapezoidProfile.Constraints(maxVel, maxAccel);
      }

      public static final class Conversions {
        public static final double kRatio = 360;
      }

      // TODO fix this from arm setpoint vel & pos to goal setpoint for inverse kinematics
      public static enum Setpoints {
        TEST_FLOATING_SETPOINT(1, 0);

        public double position;
        public double velocity;

        Setpoints(double position, double velocity) {
          this.position = position;
          this.velocity = velocity;
        }
      }
    }
  }
}
