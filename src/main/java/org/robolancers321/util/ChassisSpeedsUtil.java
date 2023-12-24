/* (C) Robolancers 2024 */
package org.robolancers321.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class ChassisSpeedsUtil {

  private ChassisSpeedsUtil() {
    // utility class
  }

  // currently not in wpilib, is set for 2024 release?
  public static ChassisSpeeds discretize(
      double vxMetersPerSecond,
      double vyMetersPerSecond,
      double omegaRadiansPerSecond,
      double dtSeconds) {
    final var desiredDeltaPose =
        new Pose2d(
            vxMetersPerSecond * dtSeconds,
            vyMetersPerSecond * dtSeconds,
            new Rotation2d(omegaRadiansPerSecond * dtSeconds));

    final var twist = new Pose2d().log(desiredDeltaPose);

    return new ChassisSpeeds(twist.dx / dtSeconds, twist.dy / dtSeconds, twist.dtheta / dtSeconds);
  }

  public static ChassisSpeeds discretize(ChassisSpeeds speeds, double dtSeconds) {
    return discretize(
        speeds.vxMetersPerSecond,
        speeds.vyMetersPerSecond,
        speeds.omegaRadiansPerSecond,
        dtSeconds);
  }

  /* trans, rot, slow
   * Case 1: false, false, false (normal speed robot-centric)
   * Case 2: true, true, true (heading control & slow field centric drive)
   * Case 3: true, false, true (no heading control & slow field centric drive)
   * Case 4: false, false, true (slow robot centric drive)
   * Case 5: true, false, false (no heading control & normal speed field centric drive)
   * Case 6: false, true, false (heading control & normal speed robot centric drive)
   * 
   * expressed as:
   * translateFieldCentric(throttle, strafe, heading).and(rotateRobotCentric(turn))[.get() | .slow()]
   * rotateRobotCentric(turn).and(translateFieldCentric(throttle, strafe, heading))[.get() | .slow()]
   * 
   * drive(
   *    slow(translateFieldCentric()),
   *    rotateRobotCentric()
   * )
   */
}
