// /* (C) Robolancers 2024 */
// package org.robolancers321.subsystems.arm.commands;

// import edu.wpi.first.wpilibj2.command.CommandBase;
// import java.util.function.DoubleSupplier;
// import org.robolancers321.Constants;
// import org.robolancers321.subsystems.arm.Arm;
// import org.robolancers321.subsystems.arm.InverseArmKinematics;
// import org.robolancers321.util.MathUtils;

// public class MoveToTunableSetpoint extends CommandBase {
//   private Arm arm;

//   private DoubleSupplier ySupplier;
//   private DoubleSupplier zSupplier;

//   public MoveToTunableSetpoint(Arm arm, DoubleSupplier ySupplier, DoubleSupplier zSupplier) {
//     this.arm = arm;

//     this.ySupplier = ySupplier;
//     this.zSupplier = zSupplier;
//   }

//   @Override
//   public void initialize() {
//     InverseArmKinematics.Output currentSetpointAngles =
//         InverseArmKinematics.calculate(this.ySupplier.getAsDouble(), this.zSupplier.getAsDouble());

//     arm.setAnchorSetpoint(currentSetpointAngles.anchor);
//     arm.setFloatingSetpoint(currentSetpointAngles.floating);
//   }

//   @Override
//   public boolean isFinished() {
//     InverseArmKinematics.Output currentSetpointAngles =
//         InverseArmKinematics.calculate(this.ySupplier.getAsDouble(), this.zSupplier.getAsDouble());

//     return (MathUtils.epsilonEquals(
//             currentSetpointAngles.anchor, arm.getAnchorAngle(), Constants.Arm.Anchor.kTolerance)
//         && MathUtils.epsilonEquals(
//             currentSetpointAngles.floating,
//             arm.getFloatingAngle(),
//             Constants.Arm.Floating.kTolerance));
//   }
// }
