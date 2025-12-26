// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.ArmSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ProfiledPositionArmCommand extends Command {
  /** Creates a new ProfiledPositionArmCommand. */

  private ArmSubsystem armSubsystem;
  private double positionDeg;

  private double ffVolts = 0.0;
  private double pidOutput = 0.0;

  public ProfiledPositionArmCommand(ArmSubsystem armSubsystem, double positionDeg) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.armSubsystem = armSubsystem;
    this.positionDeg = positionDeg;

    addRequirements(armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    armSubsystem.changeManualControlState(false);
    armSubsystem.resetProfiledPIDController();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ffVolts = armSubsystem.calculateWithFF(Math.toRadians(positionDeg), 0.0);
    pidOutput = armSubsystem.calculateWithProfile(positionDeg);

    armSubsystem.setMotor(pidOutput + ffVolts / RobotController.getBatteryVoltage());

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return armSubsystem.atProfiledGoal();
  }
}
