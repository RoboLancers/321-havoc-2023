/* (C) Robolancers 2024 */
package org.robolancers321.subsystems.arm.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.robolancers321.subsystems.arm.Arm;

public class ManualMoveAnchor extends CommandBase {
  private Arm arm;
  private double degPerSec = 2;
  private double angleOffset = degPerSec / 1000 * 20;

  private boolean reverse;

  public ManualMoveAnchor(Arm arm, boolean reverse) {
    this.arm = arm;
    this.reverse = reverse;

  }

  @Override
  public void execute() {
    if (reverse) {
      arm.setAnchorGoal(arm.getAnchorAngle() - angleOffset);
    } else {
      arm.setAnchorGoal(arm.getAnchorAngle() - angleOffset);
    }
  }
}
