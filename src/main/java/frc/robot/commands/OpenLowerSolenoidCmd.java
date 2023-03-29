// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.VacuumSubsystem;

public class OpenLowerSolenoidCmd extends CommandBase {
  // Subsystems
  private VacuumSubsystem m_vacuumSubsystem;

  /** Creates a new ToggleUpperSolenoid. */
  public OpenLowerSolenoidCmd(VacuumSubsystem vacuumSubsystem) {
    m_vacuumSubsystem = vacuumSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_vacuumSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_vacuumSubsystem.openLowerSolenoid();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_vacuumSubsystem.closeLowerSolenoid();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
