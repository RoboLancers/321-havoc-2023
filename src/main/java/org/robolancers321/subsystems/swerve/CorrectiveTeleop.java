/* (C) Robolancers 2024 */
package org.robolancers321.subsystems.swerve;

import edu.wpi.first.math.geometry.Translation2d;

public class CorrectiveTeleop {
  // based on the math here:
  // https://www.chiefdelphi.com/t/whitepaper-swerve-drive-skew-and-second-order-kinematics/416964/18
  // uses the pose exponential and small angle approximations to compensate for the continuous
  // movement in a discrete timestep

  private static final double dt = 0.2;

  public static Translation2d generateCorrectedInput(
      double inputThrottle, double inputStrafe, double inputOmega) {
    // change in angle over a period of dt assuming constant angular velocity
    double angularDisplacement = inputOmega * dt;

    // cache the relevant trig
    double sin = Math.sin(0.5 * angularDisplacement);
    double cos = Math.cos(0.5 * angularDisplacement);

    // TODO: flip strafe and throttle?
    // apply pose exponential with small angle apprximations
    double resultantThrottle = inputStrafe * sin + inputThrottle * cos;
    double resultantStrafe = inputStrafe * cos - inputThrottle * sin;

    return new Translation2d(resultantThrottle, resultantStrafe);
  }
}
