/* (C) Robolancers 2024 */
package org.robolancers321.util;

public class MathUtils {
  public static boolean epsilonEquals(double x, double y, double epsilon) {
    return Math.abs(x - y) < epsilon;
  }
}
