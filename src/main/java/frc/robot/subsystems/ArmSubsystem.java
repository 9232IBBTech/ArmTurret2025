// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ArmSubsystem. */
  
  private SparkMax armMotor;
  private RelativeEncoder armEncoder;
  private SparkMaxConfig armConfig;
  private SparkClosedLoopController armPIDController;

  public ArmSubsystem() {

    armMotor =  new SparkMax(Constants.ArmConstants.ARM_MOTOR_ID, SparkMax.MotorType.kBrushless);
    armConfig = new SparkMaxConfig();

    armEncoder = armMotor.getEncoder();

    armPIDController = armMotor.getClosedLoopController();
    
    armConfig.idleMode(IdleMode.kCoast);

    armMotor.configure(armConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
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
}
