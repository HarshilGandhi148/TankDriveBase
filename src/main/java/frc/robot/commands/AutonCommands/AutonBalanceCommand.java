// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutonCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer.Subsystems;
import frc.robot.utils.GalacPIDController;

public class AutonBalanceCommand extends CommandBase {

  GalacPIDController turnPID;
  GalacPIDController balancePID;
  /** Creates a new AutonDriveCommand. */
  public AutonBalanceCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Subsystems.driveSubsystem);
    turnPID = new GalacPIDController(0.009, 0, 0, 0.05, () -> Subsystems.driveSubsystem.getGyroYaw(), 0, 1);
    balancePID = new GalacPIDController(0.008, 0, 0, 0.05, () -> Subsystems.driveSubsystem.getGyroPitch(), 0, 3);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Subsystems.driveSubsystem.setBrakeEnabled();
    Subsystems.driveSubsystem.tankDrive(0, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Subsystems.driveSubsystem.tankDrive(balancePID.getEffort(), -turnPID.getEffort());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Subsystems.driveSubsystem.setBrakeEnabled();
    Subsystems.driveSubsystem.tankDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return balancePID.isFinished();
  }
}
