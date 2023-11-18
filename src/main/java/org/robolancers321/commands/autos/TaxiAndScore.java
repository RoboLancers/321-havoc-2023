/* (C) Robolancers 2024 */
package org.robolancers321.commands.autos;

import edu.wpi.first.wpilibj2.command.*;
import org.robolancers321.Constants;
import org.robolancers321.subsystems.arm.Arm;
import org.robolancers321.subsystems.intake.Intake;
import org.robolancers321.subsystems.swerve.Swerve;

public class TaxiAndScore extends SequentialCommandGroup {

  public TaxiAndScore(
      Arm arm,
      Swerve swerve,
      Intake intake,
      Constants.RawArmSetpoints setpoint,
      Score.ItemType type) {

    addRequirements(swerve);

    addCommands(
        new Score(arm, intake, setpoint, type),
        new ParallelRaceGroup(
            new WaitCommand(2.5),
            new RunCommand(
                () -> {
                  swerve.drive(-0.5, 0, 0, true);
                })),
        new InstantCommand(
            () -> {
              swerve.drive(0, 0, 0, true);
            })
        );
  }
}
