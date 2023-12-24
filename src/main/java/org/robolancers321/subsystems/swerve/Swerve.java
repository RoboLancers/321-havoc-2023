/* (C) Robolancers 2024 */
package org.robolancers321.subsystems.swerve;

import static java.util.Map.entry;
import static org.robolancers321.Constants.Swerve.*;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Collections;
import java.util.List;
import java.util.Map;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.robolancers321.util.SmartDashboardUtil;

public class Swerve extends SubsystemBase {
  private final List<SwerveModule> modules;

  private final AHRS gyro;

  private final Field2d field;
  private final SwerveDrivePoseEstimator poseEstimator;

  private SwerveAutoBuilder autoBuilder;
  private final Map<String, Command> pathEvents;

  public Swerve(
      SwerveModule frontLeft,
      SwerveModule frontRight,
      SwerveModule backLeft,
      SwerveModule backRight,
      AHRS gyro,
      Field2d field) {
    this.modules = List.of(frontLeft, frontRight, backLeft, backRight);

    this.gyro = gyro;
    this.field = field;

    this.poseEstimator =
        new SwerveDrivePoseEstimator(
            kSwerveKinematics, gyro.getRotation2d(), getModulePositions(), new Pose2d());

    gyro.zeroYaw();

    SmartDashboard.putData("Field", this.field);

    this.pathEvents =
        Map.ofEntries( // might inject this from RobotContainer instead
            entry("", runOnce(() -> {}))
            // . . .
            );

    this.autoBuilder =
        new SwerveAutoBuilder(
            this::getPose,
            this::resetPose,
            kTranslationConstants,
            kRotationConstants,
            this::drive,
            pathEvents,
            true,
            this);
  }

  @Override
  public void periodic() {
    this.field.setRobotPose(poseEstimator.update(gyro.getRotation2d(), getModulePositions()));

    // final var kDriveP = SmartDashboardUtil.pollOrDefault("kDriveP", Drive.kP);
    // final var kDriveI = SmartDashboardUtil.pollOrDefault("kDriveI", Drive.kI);
    // final var kDriveD = SmartDashboardUtil.pollOrDefault("kDriveP", Drive.kD);
    // final var kDriveFF = SmartDashboardUtil.pollOrDefault("kDriveFF", Drive.kFF);
    // final var kTurnP = SmartDashboardUtil.pollOrDefault("kTurnP", Turn.kP);
    // final var kTurnI = SmartDashboardUtil.pollOrDefault("kTurnI", Turn.kI);
    // final var kTurnD = SmartDashboardUtil.pollOrDefault("kTurnD", Turn.kD);
    // final var kTurnFF = SmartDashboardUtil.pollOrDefault("kTurnFF", Turn.kFF);

    modules.forEach(
        module -> {
          // module.setDrivePIDFCoeffs(
          //   kDriveP, kDriveI, kDriveD, kDriveFF);
          // module.setTurnPIDFCoeffs(
          //   kTurnP, kTurnI, kTurnD, kTurnFF);
          module.periodic();
        });

    SmartDashboard.putNumber("yaw deg", gyro.getYaw());
  }

  @Override
  public void simulationPeriodic() {

  }

  // this is a terrible tuning method
  public void updateDrivetrainPIDs() {
    final double kTransP = SmartDashboardUtil.pollOrDefault("kTransP", kTranslationConstants.kP);
    final double kTransI = SmartDashboardUtil.pollOrDefault("kTransI", kTranslationConstants.kI);
    final double kTransD = SmartDashboardUtil.pollOrDefault("kTransD", kTranslationConstants.kD);
    final double kRotP = SmartDashboardUtil.pollOrDefault("kRotP", kRotationConstants.kP);
    final double kRotI = SmartDashboardUtil.pollOrDefault("kRotI", kRotationConstants.kI);
    final double kRotD = SmartDashboardUtil.pollOrDefault("kRotD", kRotationConstants.kD);

    this.autoBuilder =
        new SwerveAutoBuilder(
            this::getPose,
            this::resetPose,
            new PIDConstants(kTransP, kTransI, kTransD),
            new PIDConstants(kRotP, kRotI, kRotD),
            this::drive,
            pathEvents,
            true,
            this);
  }

  public CommandBase goTo(Supplier<Pose2d> pose, PathConstraints constraints, boolean fieldRelative) {
    return goTo(pose.get(), constraints, fieldRelative);
  }

  public CommandBase goTo(Pose2d pose, PathConstraints constraints, boolean fieldRelative) {
    final var currPose = getPose();
    return followPathWithEvents(
        PathPlanner.generatePath(
            constraints,
            PathPoint.fromCurrentHolonomicState(currPose, getSpeeds()),
            new PathPoint((
              fieldRelative
                ? pose
                : currPose.plus( // allows passed in pose to be robot-relative
                    new Transform2d(pose.getTranslation(), Rotation2d.fromRadians(0)))).getTranslation(),
              Rotation2d.fromRadians(0), pose.getRotation())));
  }

  public CommandBase followPathWithEvents(String trajectoryName, PathConstraints constraints) {
    final var traj = PathPlanner.loadPath(trajectoryName, constraints);
    return followPathWithEvents(traj);
  }

  public CommandBase followPathWithEvents(PathPlannerTrajectory trajectory) {
    /*
    dear sir Vincent, I am very appalled and offended that you denied my suggestive
    recommendation of naming the "autobuilder" into the "Vincent special". I give
    you fair warning that I am equipped with the ability to summon the one known
    as little travis
     */

    this.field.getRobotObject().setPoses(trajectory.getStates().stream().map(s -> s.poseMeters).toList());
    return autoBuilder.followPathWithEvents(trajectory);
  }

  /**
   * Overrides the modules' angle setpoints so that the wheels form an X.
   */
  public CommandBase lockModules() {
    // caching
    final var pos45 = new SwerveModuleState(0, Rotation2d.fromDegrees(45));
    final var neg45 = new SwerveModuleState(0, Rotation2d.fromDegrees(-45));
    return run(
        () ->
            setModuleStates(
                pos45, neg45,
                neg45, pos45));
  }

  public CommandBase driveTeleop(
      DoubleSupplier throttle, DoubleSupplier strafe, DoubleSupplier turn, boolean fieldCentric) {
    return run(
        () ->
            drive(throttle.getAsDouble(), strafe.getAsDouble(), -turn.getAsDouble(), fieldCentric));
  }

  public void drive(double inputThrottle, double inputStrafe, double turn, boolean fieldRelative) {
    Translation2d correctedInput = correctDriverInput(inputThrottle, inputStrafe, turn);

    double throttle = correctedInput.getX();
    double strafe = correctedInput.getY();

    final var speeds =
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(throttle, strafe, turn, gyro.getRotation2d())
            : new ChassisSpeeds(throttle, strafe, turn);

    drive(speeds);
  }

  public void drive(ChassisSpeeds speeds) {
    final var states = kSwerveKinematics.toSwerveModuleStates(speeds);

    SwerveDriveKinematics.desaturateWheelSpeeds(states, kMaxSpeedMetersPerSecond);

    setModuleStates(states);
  }

  public void setModuleStates(SwerveModuleState state) {
    setModuleStates(
        Collections.nCopies(modules.size(), state).toArray(new SwerveModuleState[modules.size()]));
  }

  public void setModuleStates(
      SwerveModuleState frontLeft,
      SwerveModuleState frontRight,
      SwerveModuleState backLeft,
      SwerveModuleState backRight) {
    setModuleStates(
        new SwerveModuleState[] {
          frontLeft, frontRight,
          backLeft, backRight
        });
  }

  private void setModuleStates(SwerveModuleState[] states) {
    final var numModules = modules.size();
    for (int i = 0; i < numModules; i++) modules.get(i).setDesiredState(states[i]);
  }

  public Pose2d getPose() {
    return this.poseEstimator.getEstimatedPosition();
  }

  public void resetPose(Pose2d pose) {
    this.poseEstimator.resetPosition(gyro.getRotation2d(), getModulePositions(), pose);
  }

  public SwerveModulePosition[] getModulePositions() {
    return modules.stream().map(SwerveModule::getPosition).toArray(SwerveModulePosition[]::new);
  }

  public ChassisSpeeds getSpeeds() {
    return kSwerveKinematics.toChassisSpeeds(
        modules.stream().map(SwerveModule::getState).toArray(SwerveModuleState[]::new));
  }

  private Translation2d correctDriverInput(
      double inputThrottle, double inputStrafe, double inputOmega) {
    final double dt = 0.2;
    // change in angle over a period of dt assuming constant angular velocity
    final double angularDisplacement = inputOmega * dt;
    // cache the relevant trig
    final double sin = Math.sin(0.5 * angularDisplacement);
    final double cos = Math.cos(0.5 * angularDisplacement);

    // apply pose exponential with small angle approximations
    final double resultantThrottle = inputStrafe * sin + inputThrottle * cos;
    final double resultantStrafe = inputStrafe * cos - inputThrottle * sin;

    return new Translation2d(resultantThrottle, resultantStrafe);
  }
}
