/* (C) Robolancers 2024 */
package org.robolancers321.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import org.robolancers321.Constants;

public class Intake extends SubsystemBase {
  public CANSparkMax intakeMotor;
  SparkMaxPIDController intakePIDController;
  RelativeEncoder intakeEncoder;
  double targetVel = 0;

  public double velocity;

  public Intake() {
    this.intakeMotor = new CANSparkMax(Constants.Intake.kPort, MotorType.kBrushless);
    this.intakeEncoder = intakeMotor.getEncoder();
    intakePIDController = intakeMotor.getPIDController();
    this.intakeEncoder.setVelocityConversionFactor(1.0);

    intakePIDController.setD(Constants.IntakePID.kD);
    intakePIDController.setP(Constants.IntakePID.kP);
    intakePIDController.setI(Constants.IntakePID.kI);

    this.intakeMotor.setSmartCurrentLimit(20);

    SmartDashboard.putNumber("intakeKP", Constants.IntakePID.kP);
    SmartDashboard.putNumber("intakeKI", Constants.IntakePID.kI);
    SmartDashboard.putNumber("intakeKD", Constants.IntakePID.kD);
    SmartDashboard.putNumber("intakeKFF", Constants.IntakePID.kFF);
    SmartDashboard.putNumber("currVelocity", intakeEncoder.getVelocity());
    // SmartDashboard.putNumber("targetVelocity", 0);

  }

  @Override
  public void periodic() {
    // intakePIDController.setReference(velocity, ControlType.kVelocity);
    // this.intakeMotor.set(velocity);
    double kP = SmartDashboard.getNumber("intakeKP", 0.0);
    double kI = SmartDashboard.getNumber("intakeKI", 0.0);
    double kD = SmartDashboard.getNumber("intakeKD", 0.0);
    double kFF = SmartDashboard.getNumber("intakeKFF", 0.0);
    // double targetVelocity = SmartDashboard.getNumber("targetVelocity", 0.0);
    SmartDashboard.putNumber("currVelocityRPM", intakeEncoder.getVelocity());
    SmartDashboard.putNumber("targetVelocityRPM", this.targetVel);

    intakePIDController.setD(kD);
    intakePIDController.setP(kP);
    intakePIDController.setI(kI);
    intakePIDController.setFF(kFF);
  }

  public void intakeSlow() {
    setIntakeVelocity(Constants.Intake.kLowVelocity);
  }

  public void intakeFast() {
    setIntakeVelocity(Constants.Intake.kMaxVelocity);
  }

  public void outtakeSlow() {
    setIntakeVelocity(-Constants.Intake.kLowVelocity);
  }

  public void outtakeFast() {
    setIntakeVelocity(-Constants.Intake.kMaxVelocity);
  }

  public void stopIntake() {
    setIntakeVelocity(0);
  }

  public void setIntakeVelocity(double targetVelocityRPM) {
    this.targetVel = targetVelocityRPM;
    intakePIDController.setReference(targetVelocityRPM, ControlType.kVelocity);
  }
}
