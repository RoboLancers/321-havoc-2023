/* (C) Robolancers 2024 */
package org.robolancers321.subsystems.arm.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.robolancers321.Constants;
import org.robolancers321.subsystems.arm.Arm;

public class RunArm extends CommandBase {
  private Arm arm;

  public RunArm(Arm arm) {
    this.arm = arm;

    addRequirements(arm);
  }

  @Override
  public void execute() {
    double anchorFF =
        Constants.Arm.Anchor.ANCHOR_FEEDFORWARD.calculate(
            Math.toRadians(arm.getAnchorAngle() - 180), 0);
    arm.setAnchorControllerReference(arm.getAnchorSetpoint(), anchorFF);

    // double floatingFF =
    //     Constants.Arm.Floating.FF.FLOATING_FEEDFORWARD.calculate(
    //         Math.toRadians(arm.getFloatingSetpoint()), 0);
    arm.setFloatingControllerReference(arm.getFloatingSetpoint(), 0);
  }
}
