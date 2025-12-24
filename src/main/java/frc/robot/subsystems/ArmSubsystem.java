// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.MotorConstants;

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ArmSubsystem. */
  
  private SparkMax armMotor;
  private RelativeEncoder armEncoder;
  private SparkMaxConfig armConfig;
  private SparkClosedLoopController armPIDController;
  private SysIdRoutine sysId;
  private double allowedErr;

  private final MutAngle mut_AnglePosition = new MutAngle(0, 0, Radians); // CPU korumaları
  private final MutVoltage mut_AppliedVoltage = new MutVoltage(0, 0, Units.Volts);
  private final MutAngularVelocity mut_AngularVelocity = new MutAngularVelocity(0, 0, Units.RadiansPerSecond);
  private final ArmFeedforward armFF = new ArmFeedforward(2, 0.6, 0.6, 0.6); // 2.3449, 2.03, 0.49, 0.53973

  public ArmSubsystem() {

    armMotor =  new SparkMax(Constants.ArmConstants.ARM_MOTOR_ID, SparkMax.MotorType.kBrushless);
    armConfig = new SparkMaxConfig();

    armEncoder = armMotor.getEncoder();
    armEncoder.setPosition(0);

    armPIDController = armMotor.getClosedLoopController();

    armConfig.smartCurrentLimit(40)
              .voltageCompensation(12.0)
              .idleMode(IdleMode.kBrake)
              .encoder
              .positionConversionFactor(ArmConstants.GEAR_RATIO * 360) // derece cinsinden
              .velocityConversionFactor(ArmConstants.GEAR_RATIO); // rotation

    armConfig.inverted(true)
              .softLimit
              .forwardSoftLimitEnabled(true)
              .forwardSoftLimit(ArmConstants.ARM_MAX_ANGLE)
              .reverseSoftLimitEnabled(true)
              .reverseSoftLimit(ArmConstants.ARM_MIN_ANGLE);


    armConfig.closedLoop
              .p(0.045)
              .i(0.0001) // motor tick ve kotu mountlanmıs kol icin
              .d(0.01)
              .iZone(2)
              .outputRange(-1, 1);

    armConfig.closedLoop
              .maxMotion
              .maxAcceleration(3500)
              .maxVelocity(5000)
              .allowedClosedLoopError(1.5);



    armMotor.configure(armConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  
                                                                                   
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Encoder Reading: ", armEncoder.getPosition());
    SmartDashboard.putNumber("the voltage", armMotor.getBusVoltage() * armMotor.getAppliedOutput());
    // SmartDashboard.putNumber("SoftLimit", armMotor.configAccessor.softLimit.getForwardSoftLimit());
    // SmartDashboard.putBoolean("Limit hit", armMotor.getFault(SparkMax.FaultID.kSoftLimitFwd));
  }

  public void zeroEncoder() {
    armEncoder.setPosition(0);
  }

  public void setEncoderPositon(double positon) { // derece cinsinden
    armEncoder.setPosition(positon);
  }

  public void setBrakeMode() { // fren
    armConfig.idleMode(IdleMode.kBrake);

    armMotor.configure(armConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  private void setCoastMode() {
    armConfig.idleMode(IdleMode.kCoast);

    armMotor.configure(armConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  public void setMotor(double cycle) {
    armMotor.set(cycle);
  }

  public void setVoltage(Voltage volts) {
    armMotor.setVoltage(volts);
  }

  public double getPositionRad() {
    return (armEncoder.getPosition() / 360.0) * (2 * Math.PI /*/ ArmConstants.GEAR_RATIO*/);
  }

  public double getPositionRadPerSec() {
    return armEncoder.getVelocity() * (2 * Math.PI /*/ ArmConstants.GEAR_RATIO*/) / 60.0; 
  }

  public SparkMaxConfig getConfig() {
    return armConfig;
  }

  public void configMotor(SparkMaxConfig newConfig, ResetMode safeParameters, PersistMode persistParameters) { // configure problem cıkarır
    armConfig = newConfig;
    armMotor.configure(newConfig, safeParameters, persistParameters);
  }

  public void configAllowedError() { // configure problem cıkarır
    armConfig.closedLoop.maxMotion.allowedClosedLoopError(allowedErr);
    armMotor.configure(armConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void configAllowedError(double allowedErr) { // configure problem cıkarır
    armConfig.closedLoop.maxMotion.allowedClosedLoopError(allowedErr);
    armMotor.configure(armConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void setPosition(double setpointDeg) {
    // allowedErr = Math.abs(setpointDeg * ArmConstants.ALLOWED_ERROR);
    double angleRads = Math.toRadians(setpointDeg);
    double velocityRadPerSeconds = Math.PI / 2; // TODO

    double ffVolts = armFF.calculate(angleRads, velocityRadPerSeconds);

    SmartDashboard.putNumber("Target Angle Rad", angleRads);
    SmartDashboard.putNumber("Applied FF Volts", ffVolts);

    armPIDController.setReference(setpointDeg, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0, ffVolts, SparkClosedLoopController.ArbFFUnits.kVoltage);
  }

  public void setPosition(ArmFeedforward ff ,double setpointDeg) {
    // allowedErr = Math.abs(setpointDeg * ArmConstants.ALLOWED_ERROR);
    double angleRads = Math.toRadians(setpointDeg);
    double velocityRadPerSeconds = 0.0;

    double ffVolts = ff.calculate(angleRads, velocityRadPerSeconds);

    SmartDashboard.putNumber("Target Angle Rad", angleRads);
    SmartDashboard.putNumber("Applied FF Volts", ffVolts);

    armPIDController.setReference(setpointDeg, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0, ffVolts, SparkClosedLoopController.ArbFFUnits.kVoltage);
  }

  public void setVelocity(double cycle) {
    double velocityRadPerSeconds = 2 * Math.PI * cycle;
    double angleRads = 0.0;

    double ffVolts = armFF.calculate(angleRads, velocityRadPerSeconds);
    armPIDController.setReference(cycle, ControlType.kMAXMotionVelocityControl, ClosedLoopSlot.kSlot0, ffVolts, SparkClosedLoopController.ArbFFUnits.kVoltage);
  }

  public double getEncoderPosition() {
    return armEncoder.getPosition();
  }

  public double getEncoderVelocity() {
    return armEncoder.getVelocity();
  }

  public boolean atPositionRelative(double setpoint) {
    allowedErr = Math.abs(setpoint * ArmConstants.ALLOWED_ERROR);
    double err = Math.abs(setpoint - getEncoderPosition());

    return err <= allowedErr;
  }

  public boolean atPosition(double setpoint) {
    double err = Math.abs(setpoint - getEncoderPosition());

    return err <= ArmConstants.ALLOWED_ERROR_DEG;
    
  }

  public void readySysIDRoutine() {

    // TODO; Current limit
    armConfig.smartCurrentLimit(40).idleMode(IdleMode.kBrake).voltageCompensation(12.0).encoder.positionConversionFactor(ArmConstants.GEAR_RATIO).positionConversionFactor(ArmConstants.GEAR_RATIO);
    armConfig.inverted(true).softLimit.forwardSoftLimitEnabled(true).forwardSoftLimit((65)/360.0).reverseSoftLimitEnabled(true).reverseSoftLimit((21.4)/360.0);

    armMotor.configure(armConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    sysId = new SysIdRoutine(new SysIdRoutine.Config(
      Units.Volts.per(Units.Second).of(0.3),
      Units.Volts.of(1.0),
      null,
      null // otomatik record
    ), new SysIdRoutine.Mechanism((voltage) -> setVoltage(voltage),
                                                                                   log -> {log.motor("arm-motor")
                                                                                           .voltage(mut_AppliedVoltage.mut_replace(armMotor.getBusVoltage() * armMotor.getAppliedOutput(), Units.Volts))
                                                                                           .angularPosition(mut_AnglePosition.mut_replace(getPositionRad(), Radians))
                                                                                           .angularVelocity(mut_AngularVelocity.mut_replace(getPositionRadPerSec(), RadiansPerSecond));},
                                                                                   this));

  }

  public double calculateWithFF(double posRad, double velRad) {

    return armFF.calculate(posRad, velRad);
  }

  public Command sysIdQuasistaticForward() {
    System.out.println("Running Quasistatic Forward");
    return sysId.quasistatic(SysIdRoutine.Direction.kForward);
  }

  public Command sysIdQuasistaticReverse() {
    System.out.println("Running Quasistatic Reverse");
      return sysId.quasistatic(SysIdRoutine.Direction.kReverse);
  }

  public Command sysIdDynamicForward() {
    System.out.println("Running Dynamic Forward");
      return sysId.dynamic(SysIdRoutine.Direction.kForward);
  }

  public Command sysIdDynamicReverse() {
    System.out.println("Running Dynamic Reverse");
    // SmartDashboard.putBoolean("tamam", true);
      return sysId.dynamic(SysIdRoutine.Direction.kReverse);
  }
}
