/* (C) Robolancers 2024 */
package org.robolancers321.subsystems.arm.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.robolancers321.subsystems.arm.Arm;

public class ManualMoveFloating extends CommandBase {
  private Arm arm;

  private double speed = 0.2;
  private boolean reverse;

  public ManualMoveFloating(Arm arm, boolean reverse) {
    this.arm = arm;
    this.reverse = reverse;

    addRequirements(arm);
  }

  @Override
  public void execute() {
    if (reverse) {
      arm.setFloatingSpeed(-speed);
    } else {
      arm.setFloatingSpeed(speed);
    }
  }
}
