/* (C) Robolancers 2024 */
package org.robolancers321.util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SmartDashboardUtil {
  private SmartDashboardUtil() {
    // utility class
  }

  public static double pollOrDefault(String key, double defaultValue) {
    if (!SmartDashboard.containsKey(key)) SmartDashboard.putNumber(key, defaultValue);
    return SmartDashboard.getNumber(key, defaultValue);
  }
}
