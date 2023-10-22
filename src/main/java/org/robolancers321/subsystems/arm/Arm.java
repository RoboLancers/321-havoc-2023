/* (C) Robolancers 2024 */
package org.robolancers321.subsystems.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.robolancers321.Constants;

public class Arm extends SubsystemBase {
  private final CANSparkMax anchorMotor;
  private final CANSparkMax floatingMotor;

  private final RelativeEncoder anchorEncoder;
  private final RelativeEncoder floatingEncoder;

  private final SparkMaxPIDController anchorPIDController;
  private final SparkMaxPIDController floatingPIDController;

  public double anchorSetpoint;
  public double floatingSetpoint;

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

    initTuneControllers();
    initTuneControllers();
  }

  private void configureMotors() {
    anchorMotor.setInverted(Constants.Arm.Anchor.kMotorInverted);
    anchorMotor.setIdleMode(IdleMode.kBrake);
    anchorMotor.setSmartCurrentLimit(Constants.Arm.Anchor.kCurrentLimit);
    anchorMotor.enableVoltageCompensation(Constants.Arm.Anchor.kNominalVoltage);
    anchorMotor.setSoftLimit(SoftLimitDirection.kReverse, (float) Constants.Arm.Anchor.kMinAngle);
    anchorMotor.setSoftLimit(SoftLimitDirection.kForward, (float) Constants.Arm.Anchor.kMaxAngle);
    anchorMotor.enableSoftLimit(SoftLimitDirection.kReverse, Constants.Arm.Anchor.kEnableSoftLimit);
    anchorMotor.enableSoftLimit(SoftLimitDirection.kForward, Constants.Arm.Anchor.kEnableSoftLimit);

    // //limit feedback rate of useless info, give abs encoder feedback at 20 m/s. 65535 m/s is the
    // max
    // anchorMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535); //analog sensor
    // anchorMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 65535); //alternate encoder
    // anchorMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 65535); //absolute encoder
    // position
    // anchorMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 65535); //absolute encoder
    // velocity

    // //limit feedback rate of useless info, give abs encoder feedback at 20 m/s. 65535 m/s is the
    // max
    // anchorMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535); //analog sensor
    // anchorMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 65535); //alternate encoder
    // anchorMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 65535); //absolute encoder
    // position
    // anchorMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 65535); //absolute encoder
    // velocity

    floatingMotor.setInverted(Constants.Arm.Floating.kInverted);
    floatingMotor.setIdleMode(IdleMode.kBrake);
    floatingMotor.setSmartCurrentLimit(Constants.Arm.Floating.kCurrentLimit);
    floatingMotor.enableVoltageCompensation(Constants.Arm.Floating.kNominalVoltage);
    floatingMotor.setSoftLimit(
        SoftLimitDirection.kReverse, (float) Constants.Arm.Floating.kMinAngle);
    floatingMotor.setSoftLimit(
        SoftLimitDirection.kForward, (float) Constants.Arm.Floating.kMaxAngle);
    floatingMotor.enableSoftLimit(
        SoftLimitDirection.kReverse, Constants.Arm.Floating.kEnableSoftLimit);
    floatingMotor.enableSoftLimit(
        SoftLimitDirection.kForward, Constants.Arm.Floating.kEnableSoftLimit);

    // floatingMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535);
    // floatingMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 65535);
    // floatingMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 65535);
    // floatingMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 65535); //absolute encoder
    // velocity

    // floatingMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535);
    // floatingMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 65535);
    // floatingMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 65535);
    // floatingMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 65535); //absolute encoder
    // velocity

  }

  private void configureEncoders() {
    anchorEncoder.setPositionConversionFactor(Constants.Arm.Anchor.kdistancePerRotation);
    anchorEncoder.setPosition(Constants.Arm.Anchor.kZeroPosition);

    floatingEncoder.setPositionConversionFactor(Constants.Arm.Floating.kdistancePerRotation);
    floatingEncoder.setPosition(Constants.Arm.Floating.kZeroPosition);
  }

  private void configureControllers() {
    anchorPIDController.setP(Constants.Arm.Anchor.kP);
    anchorPIDController.setI(Constants.Arm.Anchor.kI);
    anchorPIDController.setD(Constants.Arm.Anchor.kD);
    anchorPIDController.setOutputRange(
        Constants.Arm.Anchor.kMinOutput, Constants.Arm.Anchor.kMaxOutput);

    floatingPIDController.setP(Constants.Arm.Floating.kP);
    floatingPIDController.setI(Constants.Arm.Floating.kI);
    floatingPIDController.setD(Constants.Arm.Floating.kD);
    floatingPIDController.setOutputRange(
        Constants.Arm.Floating.kMinOutput, Constants.Arm.Floating.kMaxOutput);
  }

  public double getAnchorAngle() {
    return anchorEncoder.getPosition();
  }

  public double getFloatingAngle() {
    // return ((-floatingEncoder.getPosition() - getAnchorAngle()) % 360.0) + 360.0;
    // return ((-floatingEncoder.getPosition() - getAnchorAngle()) % 360.0) + 360.0;
    return floatingEncoder.getPosition();
  }

  public double getAnchorVelocity() {
    return anchorEncoder.getVelocity();
  }

  public double getFloatingVelocity() {
    return floatingEncoder.getVelocity();
  }

  public void setAnchorSpeed(double speed) {
    anchorMotor.set(speed);
  }

  public void setFloatingSpeed(double speed) {
    floatingMotor.set(speed);
  }

  public void setAnchorControllerReference(double reference, double ff) {
    anchorPIDController.setReference(
        reference, ControlType.kPosition, Constants.Arm.Anchor.kPIDSlot, ff, ArbFFUnits.kVoltage);
  }

  public void setFloatingControllerReference(double reference, double ff) {
    floatingPIDController.setReference(
        reference, ControlType.kPosition, Constants.Arm.Floating.kPIDSlot, ff, ArbFFUnits.kVoltage);
  }

  public double getAnchorSetpoint() {
    return this.anchorSetpoint;
  }

  public double getFloatingSetpoint() {
    return this.floatingSetpoint;
  }

  public void setAnchorSetpoint(double position) {
    this.anchorSetpoint = position;
  }

  public void setFloatingSetpoint(double position) {
    this.floatingSetpoint = position;
  }

  private void initTuneControllers() {
    SmartDashboard.putNumber(
        "anchorSetpoint", SmartDashboard.getNumber("anchorSetpoint", getAnchorSetpoint()));
    SmartDashboard.putNumber(
        "floatingSetpoint", SmartDashboard.getNumber("floatingSetpoint", getFloatingSetpoint()));

    SmartDashboard.putNumber(
        "anchorKP", SmartDashboard.getNumber("anchorKP", Constants.Arm.Anchor.kP));
    SmartDashboard.putNumber(
        "anchorKI", SmartDashboard.getNumber("anchorKI", Constants.Arm.Anchor.kI));
    SmartDashboard.putNumber(
        "anchorKD", SmartDashboard.getNumber("anchorKD", Constants.Arm.Anchor.kD));
    SmartDashboard.putNumber(
        "anchorKG", SmartDashboard.getNumber("anchorKG", Constants.Arm.Anchor.kG));

    SmartDashboard.putNumber(
        "floatingKP", SmartDashboard.getNumber("floatingKP", Constants.Arm.Floating.kP));
    SmartDashboard.putNumber(
        "floatingKI", SmartDashboard.getNumber("floatingKI", Constants.Arm.Floating.kI));
    SmartDashboard.putNumber(
        "floatingKD", SmartDashboard.getNumber("floatingKD", Constants.Arm.Floating.kD));
  }

  private void tuneControllers() {
    double anchorSetpoint = SmartDashboard.getEntry("anchorSetpoint").getDouble(0);
    double anchorKP = SmartDashboard.getEntry("anchorKP").getDouble(0);
    double anchorKI = SmartDashboard.getEntry("anchorKI").getDouble(0);
    double anchorKD = SmartDashboard.getEntry("anchorKD").getDouble(0);
    double anchorKG = SmartDashboard.getEntry("anchorKG").getDouble(0);

    double floatingSetpoint = SmartDashboard.getEntry("floatingSetpoint").getDouble(0);
    double floatingKP = SmartDashboard.getEntry("floatingKP").getDouble(0);
    double floatingKI = SmartDashboard.getEntry("floatingKI").getDouble(0);
    double floatingKD = SmartDashboard.getEntry("floatingKD").getDouble(0);

    this.anchorPIDController.setP(anchorKP);
    this.anchorPIDController.setI(anchorKI);
    this.anchorPIDController.setD(anchorKD);
    this.anchorSetpoint =
        MathUtil.clamp(
            anchorSetpoint, Constants.Arm.Anchor.kMinAngle, Constants.Arm.Anchor.kMaxAngle);
    Constants.Arm.Anchor.ANCHOR_FEEDFORWARD = new ArmFeedforward(0, anchorKG, 0);

    this.floatingPIDController.setP(floatingKP);
    this.floatingPIDController.setI(floatingKI);
    this.floatingPIDController.setD(floatingKD);
    this.floatingSetpoint =
        MathUtil.clamp(
            floatingSetpoint, Constants.Arm.Floating.kMinAngle, Constants.Arm.Floating.kMaxAngle);

    SmartDashboard.putNumber("anchorAngle", getAnchorAngle());
    SmartDashboard.putNumber("floatingAngle", getFloatingAngle());

    SmartDashboard.putNumber("anchorOutput", this.anchorMotor.getAppliedOutput());
    SmartDashboard.putNumber("floatingOutput", this.floatingMotor.getAppliedOutput());
  }

  @Override
  public void periodic() {
    // tuneControllers();
  }
}
