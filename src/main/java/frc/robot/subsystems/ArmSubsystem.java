// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ArmSubsystem. */
  
  private SparkMax armMotor;
  private RelativeEncoder armEncoder;
  private SparkMaxConfig armConfig;
  private SparkClosedLoopController armPIDController;

  public ArmSubsystem() {

    armMotor =  new SparkMax(Constants.ArmConstants.ARM_MOTOR_ID, SparkMax.MotorType.kBrushless);
    // armConfig = new SparkMaxConfig().closedLoop.pid();

    armEncoder = armMotor.getEncoder();
    armEncoder.setPosition(0);

    armPIDController = armMotor.getClosedLoopController();

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void openArm(double position) {
    // armMotor.set();
  }

  public void closeArm(double speed) {}
}
