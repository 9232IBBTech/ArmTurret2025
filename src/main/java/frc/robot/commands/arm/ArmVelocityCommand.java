// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.ArmConstants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ArmVelocityCommand extends Command {
  /** Creates a new ArmVelocityCommand. */

  private ArmSubsystem armSubsystem;
  private double cycle;

  public ArmVelocityCommand(ArmSubsystem armSubsystem, double cycle) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.armSubsystem = armSubsystem;
    this.cycle = cycle;

    addRequirements(armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // armSubsystem.setMotor(cycle);
    // armSubsystem.setVelocity(cycle);
    armSubsystem.changeManualControlState(true);
    
    double radPerSecond = cycle * MotorConstants.NEO_MAX_RPM * (2 * Math.PI) * ArmConstants.GEAR_RATIO *(RobotController.getBatteryVoltage() / 12.0) / 60.0;

    double volts = armSubsystem.calculateWithFF(0.0, radPerSecond);
    
    armSubsystem.setVoltage(Volts.of(volts));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !armSubsystem.getSwitch();
  }
}
