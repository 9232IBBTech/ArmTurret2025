// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class StationaryArmCommand extends Command {
  /** Creates a new StattionaryArmCommand. */

  private ArmSubsystem armSubsystem;
  private final ArmFeedforward armFF = new ArmFeedforward(0, 0.7, 0.0, 0.0);

  private final MutVoltage volts = new MutVoltage(0, 0, Volts);

  public StationaryArmCommand(ArmSubsystem armSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.armSubsystem = armSubsystem;

    addRequirements(armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putBoolean("stationary", true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    armSubsystem.setPosition(armFF, armSubsystem.getEncoderPosition()); //OLMAAAAZZ
    //volts.mut_replace(armFF.calculate(armSubsystem.getPositionRad(), 0), Volts);

    //armSubsystem.setVoltage(volts);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //armSubsystem.setPosition(armSubsystem.getEncoderPosition());
    SmartDashboard.putBoolean("stationary", false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
