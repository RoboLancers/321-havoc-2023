/* (C) Robolancers 2024 */
package org.robolancers321.subsystems.arm.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.robolancers321.Constants;
import org.robolancers321.subsystems.arm.Arm;

public class MoveToSetpoint extends CommandBase {

  private double anchorPosSetpoint;
  private double anchorVelSetpoint;
  private double floatingPosSetpoint;
  private double floatingVelSetpoint;
  Arm arm;

  public MoveToSetpoint(
      Arm arm,
      Constants.Arm.Anchor.Setpoints anchorSetpoint,
      Constants.Arm.Floating.Setpoints floatingSetpoint) {
    this.anchorPosSetpoint = anchorSetpoint.position;
    this.anchorVelSetpoint = anchorSetpoint.velocity;
    this.floatingPosSetpoint = floatingSetpoint.position;
    this.floatingVelSetpoint = floatingSetpoint.position;
    this.arm = arm;

    addRequirements(arm);
  }

  @Override
  public void initialize() {
    // update setpoint Pos & Vel
    arm.periodicIO.anchorPosSetpoint = anchorPosSetpoint;
    arm.periodicIO.anchorVelSetpoint = anchorVelSetpoint;
    arm.periodicIO.floatingPosSetpoint = floatingPosSetpoint;
    arm.periodicIO.floatingVelSetpoint = floatingVelSetpoint;

    // //MOTION PROFILE
    // //create new MP with setpoints & get startTime
    // arm.periodicIO.anchorProfile = new
    // TrapezoidProfile(Constants.Arm.Anchor.MP.ANCHOR_CONSTRAINTS,
    // new TrapezoidProfile.State(anchorPosSetpoint, anchorVelSetpoint), new
    // TrapezoidProfile.State(arm.getAnchorAngle(), arm.getAnchorVelocity()));
    // arm.periodicIO.floatingProfile = new
    // TrapezoidProfile(Constants.Arm.Floating.MP.FLOATING_CONSTRAINTS,
    // new TrapezoidProfile.State(floatingPosSetpoint, floatingVelSetpoint), new
    // TrapezoidProfile.State(arm.getFloatingAngle(), arm.getFloatingVelocity()));
    // arm.periodicIO.anchorProfileStartTime = Timer.getFPGATimestamp();
    // arm.periodicIO.floatingProfileStartTime = Timer.getFPGATimestamp();

  }

  @Override
  public boolean isFinished() {
    return arm.getAnchorVelocity() == anchorVelSetpoint
        && arm.getAnchorAngle() == anchorPosSetpoint
        && arm.getFloatingVelocity() == floatingVelSetpoint
        && arm.getFloatingAngle() == floatingPosSetpoint;
  }
}
