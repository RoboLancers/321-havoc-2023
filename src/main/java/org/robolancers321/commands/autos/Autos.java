/* (C) Robolancers 2024 */
package org.robolancers321.commands.autos;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import org.robolancers321.Constants;
import org.robolancers321.subsystems.arm.Arm;
import org.robolancers321.subsystems.intake.Intake;
import org.robolancers321.subsystems.swerve.Swerve;

public class Autos {

  private SendableChooser<Command> autoChooser = new SendableChooser<>();

  private final Arm arm;

  private final Swerve swerve;

  private final Intake intake;

  public Autos(Swerve swerve, Arm arm, Intake intake) {

    this.arm = arm;
    this.swerve = swerve;
    this.intake = intake;

    this.configureAutos();
  }

  public void configureAutos() {
    Command taxiConeHigh =
        new TaxiAndScore(
            this.arm,
            this.swerve,
            this.intake,
            Constants.RawArmSetpoints.HIGH,
            Score.ItemType.CONE);
    Command taxiConeMid =
        new TaxiAndScore(
            this.arm, this.swerve, this.intake, Constants.RawArmSetpoints.MID, Score.ItemType.CONE);
    Command taxiConeShelf =
        new TaxiAndScore(
            this.arm,
            this.swerve,
            this.intake,
            Constants.RawArmSetpoints.SHELF,
            Score.ItemType.CONE);

    Command taxiCubeHigh =
        new TaxiAndScore(
            this.arm,
            this.swerve,
            this.intake,
            Constants.RawArmSetpoints.HIGH,
            Score.ItemType.CUBE);
    Command taxiCubeMid =
        new TaxiAndScore(
            this.arm, this.swerve, this.intake, Constants.RawArmSetpoints.MID, Score.ItemType.CUBE);
    Command taxiCubeShelf =
        new TaxiAndScore(
            this.arm,
            this.swerve,
            this.intake,
            Constants.RawArmSetpoints.SHELF,
            Score.ItemType.CUBE);

    this.autoChooser.addOption(taxiCubeShelf.getName(), taxiCubeShelf);
    this.autoChooser.addOption(taxiCubeMid.getName(), taxiCubeMid);
    this.autoChooser.addOption(taxiCubeHigh.getName(), taxiCubeHigh);

    this.autoChooser.addOption(taxiConeShelf.getName(), taxiConeShelf);
    this.autoChooser.addOption(taxiConeMid.getName(), taxiConeMid);
    this.autoChooser.addOption(taxiConeHigh.getName(), taxiConeHigh);
  }

  public SendableChooser<Command> getAutoChooser() {
    return this.autoChooser;
  }
}
