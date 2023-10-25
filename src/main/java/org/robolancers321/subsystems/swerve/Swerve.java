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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Collections;
import java.util.List;
import java.util.function.DoubleSupplier;
import org.robolancers321.subsystems.vision.Vision;

public class Swerve extends SubsystemBase {
  private final Field2d field;
  private final List<SwerveModule> modules;

  private final AHRS gyro;

  private final SwerveDrivePoseEstimator poseEstimator;

  private final Vision vision;

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
    this.vision = new Vision();

    this.poseEstimator =
        new SwerveDrivePoseEstimator(
            kSwerveKinematics, gyro.getRotation2d(), getModulePositions(), new Pose2d());

    gyro.zeroYaw();
  }

  @Override
  public void periodic() {
    this.field.setRobotPose(poseEstimator.update(gyro.getRotation2d(), getModulePositions()));

    var visionPoseEstimate = vision.getEstimatedGlobalPose();

    visionPoseEstimate.ifPresent(
        (estimate) -> {
          var stdDevs = vision.getEstimationStdDevs(estimate.estimatedPose.toPose2d());
          this.poseEstimator.addVisionMeasurement(
              estimate.estimatedPose.toPose2d(), Timer.getFPGATimestamp(), stdDevs);
        });

    SmartDashboard.putData("Field", this.field);

    modules.forEach(SwerveModule::updateTurnOutput);
  }

  public CommandBase drive(
      DoubleSupplier throttle, DoubleSupplier strafe, DoubleSupplier turn, boolean fieldCentric) {
    return run(
        () ->
            drive(throttle.getAsDouble(), strafe.getAsDouble(), -turn.getAsDouble(), fieldCentric));
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

  public void drive(double inputThrottle, double inputStrafe, double turn, boolean fieldRelative) {
    Translation2d correctedInput =
        CorrectiveTeleop.generateCorrectedInput(inputThrottle, inputStrafe, turn);

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

  public SwerveModulePosition[] getModulePositions() {
    return modules.stream().map(SwerveModule::getPosition).toArray(SwerveModulePosition[]::new);
  }
}
