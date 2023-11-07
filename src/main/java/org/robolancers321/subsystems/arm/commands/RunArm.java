/* (C) Robolancers 2024 */
package org.robolancers321.subsystems.arm.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.robolancers321.subsystems.arm.Arm;

public class RunArm extends CommandBase {
  private Arm arm;

  public RunArm(Arm arm) {
    this.arm = arm;

    addRequirements(arm);
  }

  @Override
  public void execute() {
    // PID
    double anchorFF = arm.calculateFeedforwards().get(0, 0);
    double floatingFF = arm.calculateFeedforwards().get(1, 0);

    arm.setAnchorVoltage(anchorFF);
    arm.setFloatingVoltage(floatingFF);

    // arm.setAnchorControllerReference(anchorFF);
    // arm.setFloatingControllerReference(floatingFF);

    // arm.anchorProfile =
    //   new TrapezoidProfile(Constants.Arm.Anchor.ANCHOR_CONSTRAINTS,
    //     new TrapezoidProfile.State(arm.anchorSetpoint, 0),
    //     new TrapezoidProfile.State(arm.getAnchorAngle(), arm.getAnchorVelocity()));

    // arm.floatingProfile =
    //   new TrapezoidProfile(
    //     Constants.Arm.Floating.FLOATING_CONSTRAINTS,
    //     new TrapezoidProfile.State(arm.floatingSetpoint, 0),
    //     new TrapezoidProfile.State(arm.getFloatingAngle(), arm.getFloatingVelocity()));

    //   var anchorState = arm.anchorProfile.calculate(0.02);
    //   var floatingState = arm.floatingProfile.calculate(0.02);

    //   double anchorFF = arm.calculateAnchorFF();
    //   double floatingFF = arm.calculateFloatingFF();

    //   arm.setAnchorControllerReference(anchorState.position, anchorFF);
    //   arm.setFloatingControllerReference(-floatingState.position, floatingFF);

    //   SmartDashboard.putNumber("anchorVelREAL", arm.getAnchorVelocity());
    //   SmartDashboard.putNumber("floatingVelREAL", arm.getFloatingVelocity());
    //   SmartDashboard.putNumber("anchorVelMP", anchorState.velocity);
    //   SmartDashboard.putNumber("floatingVelMP", floatingState.velocity);
  }
}
