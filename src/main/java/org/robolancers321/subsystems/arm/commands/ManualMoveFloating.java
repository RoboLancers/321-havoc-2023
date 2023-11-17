/* (C) Robolancers 2024 */
package org.robolancers321.subsystems.arm.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

import org.robolancers321.Constants;
import org.robolancers321.subsystems.arm.Arm;

public class ManualMoveFloating extends CommandBase {
  private Arm arm;
  private final double degPerSec = 0.001;
  private double angleOffset = degPerSec / 1000 * 20;
  //  deg/s / 1000 = deg/ms * ms/loop = deg/loop

  private boolean reverse;

  public ManualMoveFloating(Arm arm, boolean reverse) {
    this.arm = arm;
    this.reverse = reverse;

    addRequirements(arm);
  }

  @Override
  public void execute() {
    // double anchorFF = arm.calculateAnchorFF();
    // double floatingFF = arm.calculateFloatingFF();

    // SmartDashboard.putBoolean("is reversed", reverse);

    // floatingFF += reverse ? -0.00001 : 0.00001;

    // floatingFF = MathUtil.clamp(
    //   floatingFF,
    //   Constants.Arm.Floating.kMinOutput,
    //   Constants.Arm.Floating.kMaxOutput);

    // arm.setAnchorControllerReference(anchorFF);
    // arm.setFloatingSpeed(floatingFF);
    if (reverse) {
      arm.setFloatingGoal(arm.getFloatingAngle() - angleOffset);
    } else {
      arm.setFloatingGoal(arm.getFloatingAngle() + angleOffset);
    }
  }

  @Override
  public void end(boolean interrupted){
    // arm.setFloatingSetpoint(arm.getFloatingAngle());
  }
}

