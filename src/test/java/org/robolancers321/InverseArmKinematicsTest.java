/* (C) Robolancers 2024 */
package org.robolancers321;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.robolancers321.subsystems.arm.InverseArmKinematics;

class InverseArmKinematicsTest {
  // unit: inches
  private static final double y = 63;
  private static final double z = 46;

  @Test
  void testCalculations() {
    InverseArmKinematics.Output output = InverseArmKinematics.calculate(y, z);

    // +- 7 isn't preferred but workable (expected values are purely theoretical, we should test
    // with measured angles)
    assertEquals(64.77, output.anchor, 7);
    assertEquals(70.717, output.floating, 7);
  }
}
