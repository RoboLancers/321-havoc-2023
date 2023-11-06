/* (C) Robolancers 2024 */
package org.robolancers321;

import static org.robolancers321.Constants.OperatorConstants.*;
import static org.robolancers321.Constants.Swerve.*;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.server.PathPlannerServer;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.robolancers321.commands.autos.Autos;
import org.robolancers321.subsystems.arm.Arm;
import org.robolancers321.subsystems.arm.commands.RunArm;
import org.robolancers321.subsystems.intake.Intake;
import org.robolancers321.subsystems.swerve.Swerve;
import org.robolancers321.subsystems.swerve.SwerveModule;
import org.robolancers321.util.SmartDashboardUtil;

public class RobotContainer {
  private final Field2d field = new Field2d();
  private final CommandXboxController driver =
      new CommandXboxController(Constants.OperatorConstants.kDriverControllerPort);
  private final AHRS gyro = new AHRS(SPI.Port.kMXP);
  // private final Arm arm = new Arm();

  private final Swerve swerve =
      new Swerve(
          new SwerveModule(OffseasonSwerve.frontLeft),
          new SwerveModule(OffseasonSwerve.frontRight),
          new SwerveModule(OffseasonSwerve.backLeft),
          new SwerveModule(OffseasonSwerve.backRight),
          gyro,
          field);

  // private final Intake intake = new Intake();
  private final Autos autoPicker = new Autos(swerve, null, null);

  public RobotContainer() {
    PathPlannerServer.startServer(5811);

    swerve.setDefaultCommand(
        swerve.driveTeleop(this::getThrottle, this::getStrafe, this::getTurn, true));

    // this.arm.setDefaultCommand(new RunArm(arm));
    configureBindings();
  }

  private void configureBindings() {
    driver.a().onTrue(swerve.goTo(
		() -> new Pose2d(
			SmartDashboardUtil.pollOrDefault("traj translation meters", 1.0),
			0,
			new Rotation2d()), kDefaultPathConstraints));

    driver.b().onTrue(swerve.goTo(
		() -> new Pose2d(
			0,
			0,
			Rotation2d.fromDegrees(SmartDashboardUtil.pollOrDefault("traj rotation deg", 0))),
      kDefaultPathConstraints));

    driver.y().onTrue(Commands.runOnce(swerve::updateDrivetrainPIDs));

    driver.rightTrigger().whileTrue(swerve.lockModules());

    // driver.rightBumper().whileTrue(new ManualMoveFloating(arm, false));
    // driver.leftBumper().whileTrue(new ManualMoveFloating(arm, true));
    // driver.rightTrigger().whileTrue(new ManualMoveAnchor(arm, false));
    // driver.rightTrigger().whileTrue(new ManualMoveAnchor(arm, true));

    // driver.a().onTrue(new MoveToSetpoint(arm, Constants.Arm.ArmSetpoints.SHELF, isCubeMode));
    // driver.x().onTrue(new MoveToSetpoint(arm, Constants.Arm.ArmSetpoints.MID, isCubeMode));
    // driver.y().onTrue(new MoveToSetpoint(arm, Constants.Arm.ArmSetpoints.HIGH, isCubeMode));

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
    return kMaxSpeedMetersPerSecond * MathUtil.applyDeadband(driver.getLeftY(), kJoystickDeadband);
  }

  private double getStrafe() {
    return kMaxSpeedMetersPerSecond * MathUtil.applyDeadband(driver.getLeftX(), kJoystickDeadband);
  }

  private double getTurn() {
    return kMaxOmegaRadiansPerSecond
        * MathUtil.applyDeadband(driver.getRightX(), kJoystickDeadband);
  }
}
