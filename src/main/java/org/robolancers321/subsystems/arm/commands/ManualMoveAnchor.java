/* (C) Robolancers 2024 */
package org.robolancers321.subsystems.arm.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.robolancers321.subsystems.arm.Arm;

public class ManualMoveAnchor extends CommandBase {

  private Arm arm;
  private boolean reverse;
  private double offset;

  public ManualMoveAnchor(Arm arm, boolean reverse) {
    this.arm = arm;
    this.reverse = reverse;
    addRequirements(arm);
  }

  @Override
  public void execute() {
    if (reverse) {
      arm.periodicIO.anchorPosSetpoint -= offset;
    } else {
      arm.periodicIO.anchorPosSetpoint += offset;
    }
  }
}
