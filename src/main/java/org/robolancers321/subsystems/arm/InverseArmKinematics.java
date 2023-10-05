/* (C) Robolancers 2024 */
package org.robolancers321.subsystems.arm;

import org.robolancers321.Constants;
import org.robolancers321.util.RelativePlane;

public class InverseArmKinematics {

  public static final class Angles {
    private final double anchor;
    private final double floating;

    public Angles(double anchor, double floating) {
      this.floating = floating;
      this.anchor = anchor;
    }

    public double getFloatingAngle() {
      return this.floating;
    }

    public double getAnchorAngle() {
      return this.anchor;
    }

    /*
        No setters needed
    */

  }

  private static double getReach(double y, double z) {
    return Math.sqrt(Math.pow(z, 2) + Math.pow(y, 2));
  }

  private static final double anchorLength = Constants.Arm.Anchor.kAnchorLength;

  private static final double floatLength = Constants.Arm.Floating.kFloatingLength;

  // Length between base and floating

  private final RelativePlane plane;
  private final double dZ;
  private final double dY;
  private final double reach;

  public InverseArmKinematics(RelativePlane plane, double dZ, double dY) {
    this.plane = plane;
    this.dY = dY + this.plane.getYOffset();
    this.dZ = dZ;
    this.reach = getReach(this.dY, this.dZ);
  }

  private double calculateBeta() {
    double top =
        (Math.pow(this.dZ, 2)
            + Math.pow(this.dY, 2)
            - Math.pow(anchorLength, 2)
            - Math.pow(floatLength, 2));

    double bottom = (2 * anchorLength * floatLength);

    return Math.acos(top / bottom);
  }

  private static double calculateBeta(double rawZ, double rawY) {
    double top =
        (Math.pow(rawZ, 2)
            + Math.pow(rawY, 2)
            - Math.pow(anchorLength, 2)
            - Math.pow(floatLength, 2));

    double bottom = (2 * anchorLength * floatLength);

    return Math.acos(top / bottom);
  }

  private double calculateAlpha(double beta) {
    return Math.asin(this.dY / this.reach) + Math.asin(floatLength * Math.sin(beta) / this.reach);
  }

  private static double calculateAlpha(double beta, double rawY, double rawZ) {
    double reach = getReach(rawY, rawZ);

    return Math.asin(rawY / reach) + Math.asin(floatLength * Math.sin(beta) / reach);
  }

  public Angles getAngles() {
    double beta = this.calculateBeta();
    double alpha = this.calculateAlpha(beta);

    return new Angles(alpha, beta);
  }

  public static Angles getAnglesRaw(double rawZ, double rawY) {
    double beta = calculateBeta(rawZ, rawY);
    double alpha = calculateAlpha(beta, rawY, rawZ);

    return new Angles(alpha, beta);
  }
}
