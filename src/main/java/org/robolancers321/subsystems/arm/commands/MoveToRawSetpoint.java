/* (C) Robolancers 2024 */
package org.robolancers321.subsystems.arm.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.robolancers321.Constants;
import org.robolancers321.subsystems.arm.Arm;
import org.robolancers321.util.MathUtils;

public class MoveToRawSetpoint extends CommandBase {
  private Arm arm;

  private double anchorPosSetpoint;
  private double floatingPosSetpoint;

  public MoveToRawSetpoint(Arm arm, Constants.RawArmSetpoints setpoint) {
    this.arm = arm;

    this.anchorPosSetpoint = setpoint.anchor;
    this.floatingPosSetpoint = setpoint.floating;
  }

  @Override
  public void initialize() {
    arm.anchorSetpoint = anchorPosSetpoint;
    arm.floatingSetpoint = floatingPosSetpoint;

    // MOTION PROFILE
    // arm.periodicIO.anchorProfile = new TrapezoidProfile(
    //   Constants.Arm.Anchor.MP.ANCHOR_CONSTRAINTS,
    //   new TrapezoidProfile.State(anchorPosSetpoint, 0),
    //   new TrapezoidProfile.State(arm.getAnchorAngle(), arm.getAnchorVelocity()));

    // arm.periodicIO.floatingProfile = new TrapezoidProfile(
    //    Constants.Arm.Floating.MP.FLOATING_CONSTRAINTS,
    //    new TrapezoidProfile.State(floatingPosSetpoint, 0),
    //    new TrapezoidProfile.State(arm.getFloatingAngle(), arm.getFloatingVelocity()));

    // arm.periodicIO.anchorProfileStartTime = Timer.getFPGATimestamp();
    // arm.periodicIO.floatingProfileStartTime = Timer.getFPGATimestamp();
  }

  @Override
  public boolean isFinished() {
    return (MathUtils.epsilonEquals(
            anchorPosSetpoint, arm.getAnchorAngle(), Constants.Arm.Anchor.kTolerance)
        && MathUtils.epsilonEquals(
            floatingPosSetpoint, arm.getFloatingAngle(), Constants.Arm.Floating.kTolerance));
  }
}
