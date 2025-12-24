// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ArmVelocityWithPositionCommand extends Command {
  /** Creates a new ArmVelocityWithPositionCommand. */

  private ArmSubsystem armSubsystem;
  private double addition;
  private double totalAddition;
  private BooleanSupplier button;

  public ArmVelocityWithPositionCommand(
    ArmSubsystem armSubsystem,
    BooleanSupplier button,
    double addition
  ) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.armSubsystem = armSubsystem;
    this.addition = addition;
    this.button = button;

    addRequirements(armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    totalAddition = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (button.getAsBoolean()) {
      totalAddition += addition;
      armSubsystem.setPosition(armSubsystem.getEncoderPosition() + totalAddition);
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      armSubsystem.setMotor(0);
    }
    totalAddition = 0;
  } //TODO

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return armSubsystem.atPositionRelative(armSubsystem.getEncoderPosition() + totalAddition);
  }
}
