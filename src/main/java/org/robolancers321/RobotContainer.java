/* (C) Robolancers 2024 */
package org.robolancers321;

import static org.robolancers321.Constants.OperatorConstants.*;
import static org.robolancers321.Constants.Swerve.*;

import org.robolancers321.Constants.RawArmSetpoints;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import org.robolancers321.commands.autos.Autos;
import org.robolancers321.subsystems.arm.Arm;
import org.robolancers321.subsystems.arm.commands.RunArm;
import org.robolancers321.subsystems.arm.commands.ManualMoveAnchor;
import org.robolancers321.subsystems.arm.commands.ManualMoveFloating;
import org.robolancers321.subsystems.arm.commands.MoveArmSeparate;
import org.robolancers321.subsystems.arm.commands.MoveArmSeparateBackwards;
import org.robolancers321.subsystems.intake.Intake;
import org.robolancers321.subsystems.intake.commands.RunIntake;
import org.robolancers321.subsystems.intake.commands.RunOuttake;
import org.robolancers321.subsystems.swerve.Swerve;
import org.robolancers321.subsystems.swerve.SwerveModule;

public class RobotContainer {
  private final Field2d field = new Field2d();
  private final CommandXboxController driver =
      new CommandXboxController(Constants.OperatorConstants.kDriverControllerPort);
  private final CommandXboxController manipulator =
      new CommandXboxController(Constants.OperatorConstants.kManipulatorControllerPort);
  private final AHRS gyro = new AHRS(SPI.Port.kMXP);
  private final Arm arm = new Arm();

  private boolean slowMode = false;

  private final Swerve swerve =
      new Swerve(
          new SwerveModule(frontLeft),
          new SwerveModule(frontRight),
          new SwerveModule(backLeft),
          new SwerveModule(backRight),
          gyro,
          field);

  private final Intake intake = new Intake();
  private final Autos autoPicker = new Autos(swerve, arm, intake);

  public RobotContainer() {
    swerve.setDefaultCommand(swerve.drive(this::getThrottle, this::getStrafe, this::getTurn,
    true));

    this.arm.setDefaultCommand(new RunArm(arm));

    configureBindings();
  }

  private void configureBindings() {
    driver.rightBumper().onTrue(new InstantCommand(() -> slowMode = true));
    driver.rightBumper().onFalse(new InstantCommand(() -> slowMode = false));
    driver.x().onTrue(new MoveArmSeparateBackwards(arm, Constants.RawArmSetpoints.CONTRACT));
    driver.b().onTrue(new MoveArmSeparate(arm, Constants.RawArmSetpoints.SHELFCONE));
    driver.y().onTrue(new MoveArmSeparate(arm, Constants.RawArmSetpoints.SHELFCUBE));
    driver.leftTrigger().whileTrue(new RunOuttake(intake, Constants.Intake.kMaxVelocity));
    driver.rightTrigger().whileTrue(new RunIntake(intake, Constants.Intake.kMaxVelocity));


    //manipulator arm
    manipulator.b().onTrue(new MoveArmSeparateBackwards(arm, Constants.RawArmSetpoints.CONTRACT));

    manipulator.x().onTrue(new MoveArmSeparate(arm, Constants.RawArmSetpoints.MID));

    manipulator.y().onTrue(new MoveArmSeparate(arm, Constants.RawArmSetpoints.HIGH));

    manipulator.a().onTrue(new MoveArmSeparate(arm, Constants.RawArmSetpoints.SHELFCONE)); //also ground

    manipulator.leftBumper().onTrue(new MoveArmSeparate(arm, Constants.RawArmSetpoints.SHELFCUBE));

    // Trigger isMovingFast = new Trigger(() -> 
    //   Math.hypot(getStrafe(), getThrottle()) > 0.5
    // );

    // isMovingFast.whileTrue(new MoveArmSeparateBackwards(arm, Constants.RawArmSetpoints.CONTRACT));

    //manipulator manual arm 
    // manipulator.rightTrigger().whileTrue(new ManualMoveAnchor(arm, false));
    // manipulator.rightBumper().whileTrue(new ManualMoveFloating(arm, false));
    
    // manipulator.leftTrigger().whileTrue(new ManualMoveAnchor(arm, true));
    // manipulator.leftBumper().whileTrue(new ManualMoveFloating(arm, true));

    //manipulator intake
    Trigger intakeCone = new Trigger(() -> manipulator.getLeftY() > 0.2);
    Trigger outtakeCone = new Trigger(() -> manipulator.getLeftY() < -0.2);
    Trigger intakeCube = new Trigger(() -> manipulator.getRightY() > 0.2);
    Trigger outtakeCube = new Trigger(() -> manipulator.getRightY() < -0.2);

    intakeCone.whileTrue(new RunIntake(intake, Constants.Intake.kMaxVelocity));
    outtakeCone.whileTrue(new RunOuttake(intake, Constants.Intake.kMaxVelocity));

    outtakeCube.whileTrue(new RunIntake(intake, Constants.Intake.kMaxVelocity));
    intakeCube.whileTrue(new RunOuttake(intake, Constants.Intake.kMaxVelocity));
  };

  public Command getAutonomousCommand() {
    return autoPicker.getAutoChooser().getSelected();
  }

  private double getThrottle() {
    double multiplier = slowMode ? 0.1 : 1.0;

    return multiplier * kMaxSpeedMetersPerSecond * MathUtil.applyDeadband(driver.getLeftY(), kJoystickDeadband);
  }

  private double getStrafe() {
    double multiplier = slowMode ? 0.1 : 1.0;

    return multiplier * kMaxSpeedMetersPerSecond * MathUtil.applyDeadband(driver.getLeftX(), kJoystickDeadband);
  }

  private double getTurn() {
    double multiplier = slowMode ? 0.1 : 1.0;

    return multiplier * kMaxOmegaRadiansPerSecond
        * MathUtil.applyDeadband(driver.getRightX(), kJoystickDeadband);
  }
}
