/* (C) Robolancers 2024 */
package org.robolancers321.subsystems.arm.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.robolancers321.Constants;
import org.robolancers321.Constants.ArmSetpoints;
import org.robolancers321.subsystems.arm.Arm;
import org.robolancers321.util.MathUtils;

public class MoveToSetpoint extends CommandBase {
  private Arm arm;

  private double anchorSetpoint;
  private double floatingSetpoint;
  private double cubeOffset = 0;

  public MoveToSetpoint(Arm arm, ArmSetpoints setpoint, boolean isCubeMode) {
    this.arm = arm;

    this.anchorSetpoint = setpoint.getAnchor();

    if (isCubeMode) {
      this.floatingSetpoint = setpoint.getFloating() + this.cubeOffset;
    } else {
      this.floatingSetpoint = setpoint.getFloating();
    }
  }

  @Override
  public void initialize() {
    arm.setAnchorSetpoint(anchorSetpoint);
    arm.setFloatingSetpoint(floatingSetpoint);
  }

  @Override
  public boolean isFinished() {
    return (MathUtils.epsilonEquals(
            anchorSetpoint, arm.getAnchorAngle(), Constants.Arm.Anchor.kTolerance)
        && MathUtils.epsilonEquals(
            floatingSetpoint, arm.getFloatingAngle(), Constants.Arm.Floating.kTolerance));
  }
}
