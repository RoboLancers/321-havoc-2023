/* (C) Robolancers 2024 */
package org.robolancers321.subsystems.arm;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.robolancers321.Constants;

public class Arm extends SubsystemBase {
  private final CANSparkMax anchorMotor;
  private final CANSparkMax floatingMotor;

  private final RelativeEncoder anchorEncoder;
  private final AbsoluteEncoder floatingEncoder;

  private final ProfiledPIDController anchorPIDController;
  private final ProfiledPIDController floatingPIDController;
  private final DJArmFeedforward armFeedforward;
  private final JointConfig anchorConfig;
  private final JointConfig floatingConfig;

  public double anchorSetpoint;
  public double floatingSetpoint;
  public double kG = 0.044;

  // public TrapezoidProfile anchorProfile = new
  // TrapezoidProfile(Constants.Arm.Anchor.ANCHOR_CONSTRAINTS, new TrapezoidProfile.State());
  // public TrapezoidProfile floatingProfile = new
  // TrapezoidProfile(Constants.Arm.Floating.FLOATING_CONSTRAINTS, new TrapezoidProfile.State());
  // public double profileStartTime;

  public Arm() {
    this.anchorMotor = new CANSparkMax(Constants.Arm.Anchor.kAnchorPort, MotorType.kBrushless);
    this.anchorEncoder = anchorMotor.getEncoder();
    this.anchorPIDController =
        new ProfiledPIDController(
            Constants.Arm.Anchor.kP,
            Constants.Arm.Anchor.kI,
            Constants.Arm.Anchor.kD,
            Constants.Arm.Anchor.ANCHOR_CONSTRAINTS);

    this.floatingMotor =
        new CANSparkMax(Constants.Arm.Floating.kFloatingPort, MotorType.kBrushless);
    this.floatingEncoder = floatingMotor.getAbsoluteEncoder(Type.kDutyCycle);
    this.floatingPIDController =
        new ProfiledPIDController(
            Constants.Arm.Floating.kP,
            Constants.Arm.Floating.kI,
            Constants.Arm.Floating.kD,
            Constants.Arm.Floating.FLOATING_CONSTRAINTS);

    this.anchorConfig =
        new JointConfig(
            Constants.Arm.Anchor.kMass,
            Constants.Arm.Anchor.kLength,
            Constants.Arm.Anchor.kMOI,
            Constants.Arm.Anchor.kCGRadius,
            DCMotor.getNEO(1).withReduction(Constants.Arm.Anchor.kGearRatio));

    this.floatingConfig =
        new JointConfig(
            Constants.Arm.Floating.kMass,
            Constants.Arm.Floating.kLength,
            Constants.Arm.Floating.kMOI,
            Constants.Arm.Floating.kCGRadius,
            DCMotor.getNEO(1).withReduction(Constants.Arm.Floating.kGearRatio));

    this.armFeedforward = new DJArmFeedforward(anchorConfig, floatingConfig);

    configureMotors();
    configureEncoders();
    configureControllers();

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
    anchorMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535); // analog sensor
    anchorMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 65535); // alternate encoder
    anchorMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20); // absolute encoder
    anchorMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 20); // absolute encoder

    floatingMotor.setInverted(Constants.Arm.Floating.kMotorInverted);
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

    floatingMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535);
    floatingMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 65535);
    floatingMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20);
    floatingMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 20);
  }

  private void configureEncoders() {
    anchorEncoder.setPositionConversionFactor(Constants.Arm.Anchor.kdistancePerRotation);
    anchorEncoder.setVelocityConversionFactor(Constants.Arm.Anchor.kdistancePerRotation);
    // anchorEncoder.setInverted(Constants.Arm.Anchor.kEncoderInverted);
    anchorEncoder.setPosition(Constants.Arm.Anchor.kZeroPosition);

    floatingEncoder.setPositionConversionFactor(Constants.Arm.Floating.kdistancePerRotation);
    floatingEncoder.setVelocityConversionFactor(Constants.Arm.Floating.kdistancePerRotation);
    floatingEncoder.setInverted(Constants.Arm.Floating.kEncoderInverted);
    // floatingEncoder.setZeroOffset(Constants.Arm.Floating.kZeroPosition);
    // floatingEncoder.setPosition(Constants.Arm.Floating.kZeroPosition);
  }

  private void configureControllers() {
    anchorPIDController.setTolerance(Constants.Arm.Anchor.kTolerance);
    floatingPIDController.setTolerance(Constants.Arm.Floating.kTolerance);
    floatingPIDController.enableContinuousInput(0, 360);
    anchorPIDController.reset(getAnchorAngle());
    floatingPIDController.reset(getFloatingAngle());

    // changed inital setpoint to current reading rather than a setpoint from constants
    anchorPIDController.setGoal(getAnchorAngle());
    floatingPIDController.setGoal(getFloatingAngle());
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

  public boolean getAnchorAtSetpoint() {
    return anchorPIDController.atSetpoint();
  }

  public boolean getFloatingAtSetpoint() {
    return floatingPIDController.atSetpoint();
  }

  public double getAnchorSetpoint() {
    return anchorPIDController.getSetpoint().position;
  }

  public double getFloatingSetpoint() {
    return floatingPIDController.getSetpoint().position;
  }

  public void setAnchorSpeed(double speed) {
    anchorMotor.set(speed);
  }

  public void setFloatingSpeed(double speed) {
    floatingMotor.set(speed);
  }

  public void setAnchorControllerReference(double ff) {
    anchorMotor.set(anchorPIDController.calculate(getAnchorAngle()) + ff);
  }

  public void setFloatingControllerReference(double ff) {
    floatingMotor.set(floatingPIDController.calculate(getFloatingAngle()) + ff);
  }

  public void setAnchorGoal(double goal) {
    double clampSetpoint =
        MathUtil.clamp(goal, Constants.Arm.Anchor.kMinAngle, Constants.Arm.Anchor.kMaxAngle);
    anchorPIDController.setGoal(clampSetpoint);
  }

  public void setFloatingGoal(double goal) {
    double clampSetpoint =
        MathUtil.clamp(goal, Constants.Arm.Floating.kMinAngle, Constants.Arm.Floating.kMaxAngle);
    floatingPIDController.setGoal(clampSetpoint);
  }

  public void setAnchorVoltage(double voltage) {
    anchorMotor.setVoltage(voltage);
  }

  public void setFloatingVoltage(double voltage) {
    floatingMotor.setVoltage(voltage);
  }

  public double getAnchorGoal() {
    return anchorPIDController.getGoal().position;
  }

  public double getFloatingGoal() {
    return floatingPIDController.getGoal().position;
  }

  public Vector<N2> calculateFeedforwards() {
    double inputUpper = Math.toRadians(getAnchorSetpoint());
    double inputLower = Math.toRadians(getAnchorSetpoint());
    Vector<N2> angles = VecBuilder.fill(inputLower, inputUpper);
    Vector<N2> vectorFF = armFeedforward.feedforward(angles);
    return vectorFF;
  }

  public double calculateAnchorFF() {
    // anchor: 2
    // floating 1.5
    // intake: ~10
    double alpha = Math.toRadians(getAnchorAngle());
    double beta = Math.toRadians(getFloatingAngle()); // relative to horizontal
    double l1 = Constants.Arm.Anchor.kLength;
    double l2 = Constants.Arm.Floating.kLength;

    // double torqueL1 =

    double anchorFF = this.kG * (l1 * Math.cos(alpha) + l2 * Math.cos(beta));

    // if(alpha > 95 * Math.PI / 180){
    //   return 0;
    // }
    // return alpha > (85 * Math.PI / 180.0) ? 0 : anchorFF;

    if (alpha > Math.toRadians(110)) {
      return anchorFF;
    } else {
      return alpha > Math.toRadians(90) ? 0 : anchorFF;
    }
    // if(this.getAnchorSetpoint() > 90){
    //   return anchorFF;
    // }

    // return alpha > Math.toRadians(90) ? -anchorFF : anchorFF;
  }

  public double calculateFloatingFF() {
    double beta = Math.toRadians(getFloatingAngle()); // relative to horizontal
    double l2 = Constants.Arm.Floating.kLength;

    double floatingFF = this.kG * (l2 * Math.cos(beta));
    return floatingFF;
  }

  // public CommandBase moveArmTogether(RawArmSetpoints setpoint) {
  //   return runOnce(
  //           () -> {
  //             setAnchorSetpoint(setpoint.anchor);
  //             setFloatingSetpoint(setpoint.floating);
  //           })
  //       .until(() -> getAnchorAtSetpoint() && getFloatingAtSetpoint());
  // };

  // public CommandBase moveArmSeparate(RawArmSetpoints setpoint) {
  //   return Commands.sequence(
  //       run(
  //               () -> {
  //                 setFloatingSetpoint(setpoint.floating);
  //               })
  //           .until(() -> getFloatingAtSetpoint()),
  //       run(
  //               () -> {
  //                 setAnchorSetpoint(setpoint.anchor);
  //               })
  //           .until(() -> getAnchorAtSetpoint()));
  // }

  private void initTuneControllers() {
    SmartDashboard.putNumber(
        "anchorGoal", SmartDashboard.getNumber("anchorSetpoint", getAnchorGoal()));
    SmartDashboard.putNumber(
        "floatingGoal", SmartDashboard.getNumber("floatingSetpoint", getFloatingGoal()));
    SmartDashboard.putNumber(
        "anchorMaxVEL", SmartDashboard.getNumber("anchorMaxVEL", Constants.Arm.Anchor.maxVelocity));
    SmartDashboard.putNumber(
        "anchorMaxACCEL",
        SmartDashboard.getNumber("anchorMaxACCEL", Constants.Arm.Anchor.maxAcceleration));
    SmartDashboard.putNumber(
        "floatingMaxVEL",
        SmartDashboard.getNumber("floatingMaxVEL", Constants.Arm.Floating.maxVelocity));
    SmartDashboard.putNumber(
        "floatingMaxACCEL",
        SmartDashboard.getNumber("floatingMaxACCEL", Constants.Arm.Floating.maxAcceleration));

    SmartDashboard.putNumber(
        "anchorKP", SmartDashboard.getNumber("anchorKP", Constants.Arm.Anchor.kP));
    SmartDashboard.putNumber(
        "anchorKI", SmartDashboard.getNumber("anchorKI", Constants.Arm.Anchor.kI));
    SmartDashboard.putNumber(
        "anchorKD", SmartDashboard.getNumber("anchorKD", Constants.Arm.Anchor.kD));

    SmartDashboard.putNumber(
        "floatingKP", SmartDashboard.getNumber("floatingKP", Constants.Arm.Floating.kP));
    SmartDashboard.putNumber(
        "floatingKI", SmartDashboard.getNumber("floatingKI", Constants.Arm.Floating.kI));
    SmartDashboard.putNumber(
        "floatingKD", SmartDashboard.getNumber("floatingKD", Constants.Arm.Floating.kD));
    SmartDashboard.putNumber("customKG", SmartDashboard.getNumber("customKG", this.kG));

    SmartDashboard.putNumber(
        "anchorMinOutput",
        SmartDashboard.getNumber("anchorMinOutput", Constants.Arm.Anchor.kMinOutput));

    SmartDashboard.putNumber(
        "anchorMaxOutput",
        SmartDashboard.getNumber("anchorMaxOutput", Constants.Arm.Anchor.kMaxOutput));

    SmartDashboard.putNumber(
        "floatingMinOutput",
        SmartDashboard.getNumber("floatingMinOutput", Constants.Arm.Floating.kMinOutput));

    SmartDashboard.putNumber(
        "floatingMaxOutput",
        SmartDashboard.getNumber("floatingMaxOutput", Constants.Arm.Floating.kMaxOutput));
  }

  public void tuneControllers() {
    double anchorGoal = SmartDashboard.getEntry("anchorGoal").getDouble(0);
    double anchorKP = SmartDashboard.getEntry("anchorKP").getDouble(0);
    double anchorKI = SmartDashboard.getEntry("anchorKI").getDouble(0);
    double anchorKD = SmartDashboard.getEntry("anchorKD").getDouble(0);
    double customKG = SmartDashboard.getEntry("customKG").getDouble(0);
    double anchorMaxVEL = SmartDashboard.getEntry("anchorMaxVEL").getDouble(0);
    double anchorMaxACCEL = SmartDashboard.getEntry("anchorMaxACCEL").getDouble(0);
    double anchorMinOutput = SmartDashboard.getEntry("anchorMinOutput").getDouble(0);
    double anchorMaxOutput = SmartDashboard.getEntry("anchorMaxOutput").getDouble(0);

    double floatingGoal = SmartDashboard.getEntry("floatingGoal").getDouble(0);
    double floatingKP = SmartDashboard.getEntry("floatingKP").getDouble(0);
    double floatingKI = SmartDashboard.getEntry("floatingKI").getDouble(0);
    double floatingKD = SmartDashboard.getEntry("floatingKD").getDouble(0);
    double floatingMaxVel = SmartDashboard.getEntry("floatingMaxVEL").getDouble(0);
    double floatingMaxAccel = SmartDashboard.getEntry("floatingMaxACCEL").getDouble(0);

    double floatingMinOutput = SmartDashboard.getEntry("floatingMinOutput").getDouble(0);
    double floatingMaxOutput = SmartDashboard.getEntry("floatingMaxOutput").getDouble(0);

    TrapezoidProfile.Constraints ANCHOR_CONSTRAINTS =
        new TrapezoidProfile.Constraints(anchorMaxVEL, anchorMaxACCEL);
    TrapezoidProfile.Constraints FLOATING_CONSTRAINTS =
        new TrapezoidProfile.Constraints(floatingMaxVel, floatingMaxAccel);

    Constants.Arm.Anchor.kMinOutput = anchorMinOutput;
    Constants.Arm.Floating.kMinOutput = floatingMinOutput;
    Constants.Arm.Anchor.kMaxOutput = anchorMaxOutput;
    Constants.Arm.Floating.kMaxOutput = floatingMaxOutput;

    this.anchorPIDController.setConstraints(ANCHOR_CONSTRAINTS);
    this.anchorPIDController.setPID(anchorKP, anchorKI, anchorKD);
    setAnchorGoal(anchorGoal);

    this.kG = customKG;

    this.floatingPIDController.setConstraints(FLOATING_CONSTRAINTS);
    this.floatingPIDController.setPID(floatingKP, floatingKI, floatingKD);
    setFloatingGoal(floatingGoal);

    SmartDashboard.putNumber("anchorVelR", getAnchorVelocity());
    SmartDashboard.putNumber("floatingVelR", getFloatingVelocity());

    SmartDashboard.putNumber("anchorVelMP", anchorPIDController.getVelocityError());
    SmartDashboard.putNumber("floatingVelRMP", floatingPIDController.getVelocityError());

    SmartDashboard.putNumber("anchorOutput", this.anchorMotor.getAppliedOutput());
    SmartDashboard.putNumber("floatingOutput", this.floatingMotor.getAppliedOutput());
  }

  @Override
  public void periodic() {

    // double z = SmartDashboard.getEntry("z").getDouble(0);
    // double y = SmartDashboard.getEntry("y").getDouble(0);

    // InverseArmKinematics.Output angles = InverseArmKinematics.calculate(y - 17, z+12);

    // SmartDashboard.putNumber("anchorAngleIK", angles.anchor);
    // SmartDashboard.putNumber("floatingAngleIK", angles.anchor - angles.floating);

    SmartDashboard.putNumber("anchorAngle", getAnchorAngle());
    SmartDashboard.putNumber("floatingAngle", getFloatingAngle());

    // tuneControllers();
  }
}
