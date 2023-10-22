/* (C) Robolancers 2024 */
package org.robolancers321.subsystems.arm;

import org.robolancers321.Constants;

public class InverseArmKinematics {
  public static final class Output {
    public final double anchor;
    public final double floating;

    public Output(double anchor, double floating) {
      this.floating = floating;
      this.anchor = anchor;
    }
  }

  private static final double anchorLength = Constants.Arm.Anchor.kAnchorLength;
  private static final double floatLength = Constants.Arm.Floating.kFloatingLength;

  private static double calculateBeta(double y, double z) {
    double top =
        (Math.pow(z, 2) + Math.pow(y, 2) - Math.pow(anchorLength, 2) - Math.pow(floatLength, 2));

    double bottom = (2 * anchorLength * floatLength);

    return Math.acos(top / bottom);
  }

  private static double calculateAlpha(double beta, double y, double z) {
    double reach = Math.sqrt(y * y + z * z);

    return Math.asin(y / reach) + Math.asin(floatLength * Math.sin(beta) / reach);
  }

  public static Output calculate(double y, double z) {
    double beta = Math.toDegrees(calculateBeta(y, z));
    double alpha = Math.toDegrees(calculateAlpha(beta, y, z));

    return new Output(alpha, beta);
  }
}
