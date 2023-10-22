/* (C) Robolancers 2024 */
package org.robolancers321.commands.autos;

import edu.wpi.first.wpilibj2.command.*;
import org.robolancers321.Constants;
import org.robolancers321.subsystems.arm.Arm;
import org.robolancers321.subsystems.arm.commands.MoveToRawSetpoint;
import org.robolancers321.subsystems.intake.Intake;

public class Score extends SequentialCommandGroup {

  public enum ItemType {
    CONE,
    CUBE;
  }

  public Score(Arm arm, Intake intake, Constants.RawArmSetpoints setpoint, ItemType type) {

    addRequirements(intake);
    addCommands(
        new MoveToRawSetpoint(arm, setpoint),
        new ParallelRaceGroup(
            new WaitCommand(1.0),
            new RunCommand(type == ItemType.CONE ? intake::outtakeSlow : intake::outtakeFast)),
        new InstantCommand(intake::stopIntake));
  }
}
