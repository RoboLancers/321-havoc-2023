/* (C) Robolancers 2024 */
package frc.robot.subsystems.swerve;

import static frc.robot.Constants.Swerve.*;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModule {
  private final String id;

  private final CANSparkMax driveMotor;
  private final CANSparkMax turnMotor;

  private final RelativeEncoder driveEncoder;
  private final CANCoder turnEncoder;

  private final SparkMaxPIDController driveController;
  private final PIDController turnController;

  private SwerveModuleState desiredState;

  public SwerveModule(ModuleConfig config) {
    this.id = config.id;

    this.driveMotor = new CANSparkMax(config.kDriveId, MotorType.kBrushless);
    this.turnMotor = new CANSparkMax(config.kTurnId, MotorType.kBrushless);

    this.driveEncoder = driveMotor.getEncoder();
    this.turnEncoder = new CANCoder(config.kTurnEncoderId);

    this.driveController = driveMotor.getPIDController();
    this.turnController = new PIDController(Turn.kP, Turn.kI, Turn.kD);

    configMotors(config.driveIsInverted, config.turnIsInverted);
    configEncoders(config.magOffsetDeg);
    configControllers();
  }

  public void periodic() {
    SmartDashboard.putNumber(id + " currVeloMetersPerSecond", getState().speedMetersPerSecond);

    SmartDashboard.putNumber(
        id + " currAngleDeg",
        getState().angle.getDegrees());

    SmartDashboard.putNumber(id + " targVeloMetersPerSecond", desiredState.speedMetersPerSecond);
    SmartDashboard.putNumber(id + " targAngleDeg", desiredState.angle.getDegrees());

    final double turnOutput = turnController.calculate(turnEncoder.getAbsolutePosition());
    turnMotor.set(-MathUtil.clamp(turnOutput, -1.0, 1.0));
  }

  public void setDesiredState(SwerveModuleState state) {
    final var optimizedState = SwerveModuleState.optimize(state, new Rotation2d(turnEncoder.getAbsolutePosition()));

    // addresses skew/drift at the module level
    // optimizedState.speedMetersPerSecond *=
    // state.angle.minus(getState().angle).getCos();

    driveController.setReference(
        optimizedState.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity);
    turnController.setSetpoint(optimizedState.angle.getRadians());

    this.desiredState = optimizedState;
  }

  public void setDrivePIDFCoeffs(double p, double i, double d, double f) {
    this.driveController.setP(p);
    this.driveController.setI(i);
    this.driveController.setD(d);
    this.driveController.setFF(f);
  }

  public void setTurnPIDCoeffs(double p, double i, double d) {
    this.turnController.setPID(p, i, d);
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        driveEncoder.getPosition(), Rotation2d.fromRadians(turnEncoder.getAbsolutePosition()));
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(
        driveEncoder.getVelocity() * kRPMToMetersPerSecond, Rotation2d.fromRadians(turnEncoder.getAbsolutePosition()));
  }

  private void configMotors(boolean driveIsInverted, boolean turnIsInverted) {
    driveMotor.setInverted(driveIsInverted);
    turnMotor.setInverted(turnIsInverted);

    driveMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    turnMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

    driveMotor.setSmartCurrentLimit(40);
    turnMotor.setSmartCurrentLimit(40);

    driveMotor.enableVoltageCompensation(12);
    turnMotor.enableVoltageCompensation(12);

    driveMotor.burnFlash();
    turnMotor.burnFlash();
  }

  private void configEncoders(double magOffsetDeg) {
    final var config = kCANCoderConfig;
    config.magnetOffsetDegrees = magOffsetDeg;

    turnEncoder.configAllSettings(config);
    // driveEncoder.setVelocityConversionFactor(kRPMToMetersPerSecond);
  }

  private void configControllers() {
    setDrivePIDFCoeffs(Drive.kP, Drive.kI, Drive.kD, Drive.kFF);
    turnController.enableContinuousInput(-Math.PI, Math.PI);
  }
}
