/* (C) Robolancers 2024 */
package org.robolancers321.util;

public class RelativePlane {

  private final String plane;
  private final double yOffset;

  public RelativePlane(String plane, double yOffset) {
    this.plane = plane;
    this.yOffset = yOffset;
  }

  public String getPlane() {
    return this.plane;
  }

  public double getYOffset() {
    return this.yOffset;
  }
}
