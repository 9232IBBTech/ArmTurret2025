// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ArmSubsystem. */
  
  private SparkMax armMotor;
  private RelativeEncoder armEncoder;
  private SparkMaxConfig armConfig;
  private SparkClosedLoopController armPIDController;
  private SysIdRoutine sysId;

  private final MutAngle mut_AnglePosition = new MutAngle(0, 0, Radians);
  private final MutVoltage mut_AppliedVoltage = new MutVoltage(0, 0, Units.Volts);
  private final MutAngularVelocity mut_AngularVelocity = new MutAngularVelocity(0, 0, Units.RadiansPerSecond);

  public ArmSubsystem() {

    armMotor =  new SparkMax(Constants.ArmConstants.ARM_MOTOR_ID, SparkMax.MotorType.kBrushless);
    armConfig = new SparkMaxConfig();

    armEncoder = armMotor.getEncoder();
    armEncoder.setPosition(0.059523817151785);

    armPIDController = armMotor.getClosedLoopController();
    
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

  public void setEncoderPositon(double positon) {
    armEncoder.setPosition(positon);
  }

  public void setBrakeMode() { // fren
    armConfig = new SparkMaxConfig();
    armConfig.idleMode(IdleMode.kBrake);

    armMotor.configure(armConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  private void setCoastMode() {
    armConfig = new SparkMaxConfig();
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
    return armEncoder.getPosition() * (2 * Math.PI /*/ ArmConstants.GEAR_RATIO*/);
  }

  public double getPositionRadPerSec() {
    return armEncoder.getVelocity() * (2 * Math.PI /*/ ArmConstants.GEAR_RATIO*/) / 60.0; 
  }

  public void setPosition(double setpoint) {
    armPIDController.setReference(setpoint, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0);
  }

  public double getEncoderPosition() {
    return armEncoder.getPosition();
  }

  public double getEncoderVelocity() {
    return armEncoder.getVelocity();
  }

  public boolean atPosition(double setpoint) {
    double allowedErr = setpoint * ArmConstants.ALLOWED_ERROR;
    double err = Math.abs(setpoint - getEncoderPosition());

    return err <= allowedErr;
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
