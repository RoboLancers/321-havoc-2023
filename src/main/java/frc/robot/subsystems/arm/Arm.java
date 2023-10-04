/* (C) Robolancers 2024 */
package frc.robot.subsystems.arm;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
  public CANSparkMax anchorMotor;
  public CANSparkMax floatingMotor;

  public RelativeEncoder anchorEncoder;
  public RelativeEncoder floatingEncoder;

  public SparkMaxPIDController anchorPIDController;
  public SparkMaxPIDController floatingPIDController;

  public PeriodicIO periodicIO;

  public Arm() {
    this.anchorMotor = new CANSparkMax(Constants.Arm.Anchor.kAnchorPort, MotorType.kBrushless);
    this.anchorEncoder = anchorMotor.getEncoder();
    this.anchorPIDController = this.anchorMotor.getPIDController();

    this.floatingMotor =
        new CANSparkMax(Constants.Arm.Floating.kFloatingPort, MotorType.kBrushless);
    this.floatingEncoder = floatingMotor.getEncoder();
    this.floatingPIDController = this.floatingMotor.getPIDController();

    configureMotors();
    configureEncoders();
    configureControllers();

    this.periodicIO = new PeriodicIO();
  }

  public void configureMotors() {
    anchorMotor.setInverted(Constants.Arm.Anchor.kInverted);
    anchorMotor.setIdleMode(IdleMode.kBrake);
    anchorMotor.setSmartCurrentLimit(Constants.Arm.Anchor.kCurrentLimit);
    // if battery is over 12 V then clamp motors at 12 V
    anchorMotor.enableVoltageCompensation(12.0);
    anchorMotor.setSoftLimit(SoftLimitDirection.kReverse, (float) Constants.Arm.Anchor.kMinAngle);
    anchorMotor.setSoftLimit(SoftLimitDirection.kForward, (float) Constants.Arm.Anchor.kMaxAngle);
    anchorMotor.enableSoftLimit(SoftLimitDirection.kReverse, Constants.Arm.Anchor.kEnableSoftLimit);
    anchorMotor.enableSoftLimit(SoftLimitDirection.kForward, Constants.Arm.Anchor.kEnableSoftLimit);

    floatingMotor.setInverted(Constants.Arm.Floating.kInverted);
    floatingMotor.setIdleMode(IdleMode.kBrake);
    floatingMotor.setSmartCurrentLimit(Constants.Arm.Floating.kCurrentLimit);
    floatingMotor.enableVoltageCompensation(12.0);
    floatingMotor.setSoftLimit(
        SoftLimitDirection.kReverse, (float) Constants.Arm.Floating.kMinAngle);
    floatingMotor.setSoftLimit(
        SoftLimitDirection.kForward, (float) Constants.Arm.Floating.kMaxAngle);
    floatingMotor.enableSoftLimit(
        SoftLimitDirection.kReverse, Constants.Arm.Floating.kEnableSoftLimit);
    floatingMotor.enableSoftLimit(
        SoftLimitDirection.kForward, Constants.Arm.Floating.kEnableSoftLimit);
  }

  public void configureEncoders() {
    anchorEncoder.setPositionConversionFactor(Constants.Arm.Anchor.Conversions.kRatio);
    anchorEncoder.setPosition(Constants.Arm.Anchor.kZeroPosition);

    floatingEncoder.setPositionConversionFactor(Constants.Arm.Floating.Conversions.kRatio);
    floatingEncoder.setPosition(Constants.Arm.Floating.kZeroPosition);

    // determine velocity by delta(x)/delta(t)
  }

  public void configureControllers() {
    anchorPIDController.setP(Constants.Arm.Anchor.PID.kP);
    anchorPIDController.setI(Constants.Arm.Anchor.PID.kI);
    anchorPIDController.setD(Constants.Arm.Anchor.PID.kD);
    anchorPIDController.setOutputRange(
        Constants.Arm.Anchor.kMinOutput, Constants.Arm.Anchor.kMaxOutput);

    floatingPIDController.setP(Constants.Arm.Floating.PID.kP);
    floatingPIDController.setI(Constants.Arm.Floating.PID.kI);
    floatingPIDController.setD(Constants.Arm.Floating.PID.kD);
    floatingPIDController.setOutputRange(
        Constants.Arm.Floating.kMinOutput, Constants.Arm.Floating.kMaxOutput);
  }

  public double getAnchorAngle() {
    return anchorEncoder.getPosition();
  }

  public double getFloatingAngle() {
    return floatingEncoder.getPosition();
  }

  public double getAnchorVelocity() {
    return anchorEncoder.getVelocity();
  }

  public double getFloatingVelocity() {
    return floatingEncoder.getVelocity();
  }

  public static class PeriodicIO {
    public double anchorPosSetpoint = Constants.Arm.Anchor.kZeroPosition;
    public double floatingPosSetpoint = Constants.Arm.Floating.kZeroPosition;
    public double anchorVelSetpoint = 0.0;
    public double floatingVelSetpoint = 0.0;
    public double anchorFF = 0.0;
    public double floatingFF = 0.0;

    // MOTION PROFILE
    // public TrapezoidProfile anchorProfile = new
    // TrapezoidProfile(Constants.Arm.Anchor.MP.ANCHOR_CONSTRAINTS, new TrapezoidProfile.State());
    // public TrapezoidProfile floatingProfile = new
    // TrapezoidProfile(Constants.Arm.Floating.MP.FLOATING_CONSTRAINTS, new
    // TrapezoidProfile.State());
    // public double anchorProfileStartTime = 0.0;
    // public double floatingProfileStartTime = 0.0;
  }

  @Override
  public void periodic() {
    // cal FF
    periodicIO.anchorFF =
        Constants.Arm.Anchor.FF.ANCHOR_FEEDFORWARD.calculate(
            periodicIO.anchorPosSetpoint, periodicIO.anchorVelSetpoint);
    periodicIO.floatingFF =
        Constants.Arm.Floating.FF.FLOATING_FEEDFORWARD.calculate(
            periodicIO.floatingPosSetpoint, periodicIO.floatingVelSetpoint);

    // set FF and setpoint
    anchorPIDController.setReference(
        periodicIO.anchorPosSetpoint,
        ControlType.kPosition,
        Constants.Arm.Anchor.PID.kSlot,
        periodicIO.anchorFF,
        SparkMaxPIDController.ArbFFUnits.kVoltage);
    floatingPIDController.setReference(
        periodicIO.floatingPosSetpoint,
        ControlType.kPosition,
        Constants.Arm.Floating.PID.kSlot,
        periodicIO.floatingFF,
        SparkMaxPIDController.ArbFFUnits.kVoltage);

    // MOTION PROFILE

    // //cal Pos & Vel at time t of MP
    // TrapezoidProfile.State anchorProfileState =
    // periodicIO.anchorProfile.calculate(Timer.getFPGATimestamp() -
    // periodicIO.anchorProfileStartTime);
    // TrapezoidProfile.State floatingProfileState =
    // periodicIO.floatingProfile.calculate(Timer.getFPGATimestamp() -
    // periodicIO.floatingProfileStartTime);

    // //cal FF using Pos & Vel above
    // periodicIO.anchorFF =
    // Constants.Arm.Anchor.FF.ANCHOR_FEEDFORWARD.calculate(anchorProfileState.position,
    // anchorProfileState.velocity);
    // periodicIO.floatingFF =
    // Constants.Arm.Floating.FF.FLOATING_FEEDFORWARD.calculate(floatingProfileState.position,
    // floatingProfileState.velocity);

    // //set PIDControllers to MP setpoint
    // floatingPIDController.setReference(
    //   floatingProfileState.position,
    //   ControlType.kPosition,
    //   Constants.Arm.Floating.PID.kSlot,
    //   periodicIO.floatingFF,
    //   SparkMaxPIDController.ArbFFUnits.kVoltage);
    // anchorPIDController.setReference(
    //   anchorProfileState.position,
    //   ControlType.kPosition,
    //   Constants.Arm.Anchor.PID.kSlot,
    //   periodicIO.anchorFF,
    //   SparkMaxPIDController.ArbFFUnits.kVoltage);

  }
}
