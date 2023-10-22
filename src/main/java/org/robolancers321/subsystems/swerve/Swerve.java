/* (C) Robolancers 2024 */
package org.robolancers321.subsystems.swerve;

import static org.robolancers321.Constants.Swerve.*;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Collections;
import java.util.List;
import java.util.function.DoubleSupplier;

public class Swerve extends SubsystemBase {
  private final Field2d field;
  private final List<SwerveModule> modules;

  private final AHRS gyro;

  private final SwerveDrivePoseEstimator poseEstimator;

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
  }

  @Override
  public void periodic() {
    this.field.setRobotPose(poseEstimator.update(gyro.getRotation2d(), getModulePositions()));

    modules.forEach(SwerveModule::periodic);
  }

  public CommandBase driveTeleop(
      DoubleSupplier throttle, DoubleSupplier strafe, DoubleSupplier turn, boolean fieldCentric) {
    return run(
        () ->
            drive(throttle.getAsDouble(), strafe.getAsDouble(), -turn.getAsDouble(), fieldCentric));
  }

  public void drive(double inputThrottle, double inputStrafe, double turn, boolean fieldRelative) {
    Translation2d correctedInput =
      correctDriverInput(inputThrottle, inputStrafe, turn);
      // CorrectiveTeleop.generateCorrectedInput(inputThrottle, inputStrafe, turn);

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

  public void setModuleStates(SwerveModuleState[] states) {
    for (int i = 0; i < modules.size(); i++) modules.get(i).setDesiredState(states[i]);
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

  private Translation2d correctDriverInput(
      double inputThrottle, double inputStrafe, double inputOmega) {
	final double dt = 0.2;
    // change in angle over a period of dt assuming constant angular velocity
    double angularDisplacement = inputOmega * dt;

    // cache the relevant trig
    double sin = Math.sin(0.5 * angularDisplacement);
    double cos = Math.cos(0.5 * angularDisplacement);

    // TODO: flip strafe and throttle?
    // apply pose exponential with small angle approximations
    double resultantThrottle = inputStrafe * sin + inputThrottle * cos;
    double resultantStrafe = inputStrafe * cos - inputThrottle * sin;

    return new Translation2d(resultantThrottle, resultantStrafe);
  }
}
