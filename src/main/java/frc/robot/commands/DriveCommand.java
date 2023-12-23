// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.RobotContainer.Subsystems;
import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.Command;

public class DriveCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DoubleSupplier m_forward;
  private final DoubleSupplier m_rotation;

  public DriveCommand(DoubleSupplier forward, DoubleSupplier rotation){
    m_forward = forward;
    m_rotation = rotation;                                                                                                                                                                                                                                                                                                                                                                                                     
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Subsystems.driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Subsystems.driveSubsystem.setBrakeEnabled();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Subsystems.driveSubsystem.tankDrive(m_forward.getAsDouble(), -m_rotation.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
