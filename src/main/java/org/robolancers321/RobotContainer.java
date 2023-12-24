/* (C) Robolancers 2024 */
package org.robolancers321;

import static org.robolancers321.Constants.OperatorConstants.kJoystickDeadband;
import static org.robolancers321.Constants.OperatorConstants.kManipulatorControllerPort;
import static org.robolancers321.Constants.OperatorConstants.kSlowModeCoeff;
import static org.robolancers321.Constants.Swerve.kDefaultPathConstraints;
import static org.robolancers321.Constants.Swerve.kMaxOmegaRadiansPerSecond;
import static org.robolancers321.Constants.Swerve.kMaxSpeedMetersPerSecond;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.robolancers321.Constants.Swerve.OffseasonSwerve;
import org.robolancers321.commands.autos.Autos;
import org.robolancers321.subsystems.arm.Arm;
import org.robolancers321.subsystems.arm.commands.MoveArmSeparate;
import org.robolancers321.subsystems.intake.Intake;
import org.robolancers321.subsystems.intake.commands.RunIntake;
import org.robolancers321.subsystems.intake.commands.RunOuttake;
import org.robolancers321.subsystems.swerve.Swerve;
import org.robolancers321.subsystems.swerve.SwerveModule;
import org.robolancers321.util.SmartDashboardUtil;

public class RobotContainer {
  private final Field2d field = new Field2d();
  private final CommandXboxController driver =
      new CommandXboxController(Constants.OperatorConstants.kDriverControllerPort);
  private final CommandXboxController manipulator =
      new CommandXboxController(kManipulatorControllerPort);
  private final AHRS gyro = new AHRS(SPI.Port.kMXP);
  private final Arm arm = new Arm();

  private final Swerve swerve =
      new Swerve(
          new SwerveModule(OffseasonSwerve.frontLeft),
          new SwerveModule(OffseasonSwerve.frontRight),
          new SwerveModule(OffseasonSwerve.backLeft),
          new SwerveModule(OffseasonSwerve.backRight),
          gyro,
          field);

  private final Intake intake = new Intake();
  private final Autos autoPicker = new Autos(swerve, null, null);

  public RobotContainer() {
    swerve.setDefaultCommand(
        // swerve.run(() -> swerve.setModuleStates(new SwerveModuleState(
        //   SmartDashboardUtil.pollOrDefault("target module velocity metersPerSecond", 0.0),
        //   Rotation2d.fromDegrees(
        //     SmartDashboardUtil.pollOrDefault("target module azimuth deg", 0.0))
        // )))
        swerve.driveTeleop(this::getThrottle, this::getStrafe, this::getTurn, true));

    // this.arm.setDefaultCommand(new RunArm(arm));
    configureBindings();
  }

  private void configureBindings() {
    manipulator.b().onTrue(new MoveArmSeparate(arm, Constants.RawArmSetpoints.CONTRACT));
    manipulator.x().onTrue(new MoveArmSeparate(arm, Constants.RawArmSetpoints.MID));
    manipulator.y().onTrue(new MoveArmSeparate(arm, Constants.RawArmSetpoints.HIGH));

    Trigger intakeCone = new Trigger(() -> manipulator.getLeftY() > 0.2);
    Trigger outtakeCone = new Trigger(() -> manipulator.getLeftY() < -0.2);
    Trigger intakeCube = new Trigger(() -> manipulator.getRightY() > 0.2);
    Trigger outtakeCube = new Trigger(() -> manipulator.getRightY() < -0.2);

    intakeCone.whileTrue(new RunIntake(intake, Constants.Intake.kMaxVelocity));
    outtakeCone.whileTrue(new RunOuttake(intake, Constants.Intake.kMaxVelocity));

    outtakeCube.whileTrue(new RunIntake(intake, 0.4 * Constants.Intake.kMaxVelocity));
    intakeCube.whileTrue(new RunOuttake(intake, 0.4 * Constants.Intake.kMaxVelocity));

    // stateless slow mode
    driver
        .rightBumper()
        .whileTrue(
            swerve.driveTeleop(
                () -> kSlowModeCoeff * getThrottle(),
                () -> kSlowModeCoeff * getStrafe(),
                () -> kSlowModeCoeff * getTurn(),
                true));

    driver
        .a()
        .onTrue(
            swerve.goTo(
                () ->
                    new Pose2d(
                        SmartDashboardUtil.pollOrDefault("traj translation meters", 1.0),
                        0,
                        new Rotation2d()),
                kDefaultPathConstraints, false));

    driver
        .b()
        .onTrue(
            swerve.goTo(
                () ->
                    new Pose2d(
                        0,
                        0,
                        Rotation2d.fromDegrees(
                            SmartDashboardUtil.pollOrDefault("traj rotation deg", 0))),
                kDefaultPathConstraints, false));

    driver.y().onTrue(Commands.runOnce(swerve::updateDrivetrainPIDs));

    driver.rightTrigger().whileTrue(swerve.lockModules());

    // driver.rightBumper().whileTrue(new ManualMoveFloating(arm, false));
    // driver.leftBumper().whileTrue(new ManualMoveFloating(arm, true));
    // driver.rightTrigger().whileTrue(new ManualMoveAnchor(arm, false));
    // driver.rightTrigger().whileTrue(new ManualMoveAnchor(arm, true));

    // driverController.start().onTrue(new InstantCommand(() -> {
    //   if(isCubeMode == false){
    //     isCubeMode = true;
    //   } else {
    //     isCubeMode = false;
    //   }
    // SmartDashboard.putBoolean("isCubeMode", isCubeMode);
  }

  public Command getAutonomousCommand() {
    return autoPicker.getAutoChooser().getSelected();
  }

  private double getThrottle() {
    return
        kMaxSpeedMetersPerSecond
        * MathUtil.applyDeadband(driver.getLeftY(), kJoystickDeadband);
  }

  private double getStrafe() {
    return
        kMaxSpeedMetersPerSecond
        * MathUtil.applyDeadband(driver.getLeftX(), kJoystickDeadband);
  }

  private double getTurn() {
    return
        kMaxOmegaRadiansPerSecond
        * MathUtil.applyDeadband(driver.getRightX(), kJoystickDeadband);
  }
}
