// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;

public class AutoOneCmd extends CommandBase {
  // Subsystems
  ArmSubsystem m_armSubsystem;
  DriveSubsystem m_driveSubsystem;
  

  private final Timer m_autoTimer = new Timer();
  
  /** Creates a new AutoOneCmd. */
  public AutoOneCmd(ArmSubsystem armSubsystem, DriveSubsystem driveSubsystem) {
    m_armSubsystem = armSubsystem;
    m_driveSubsystem = driveSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_armSubsystem);
    addRequirements(m_driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_autoTimer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println(m_autoTimer.get());

    // Calibrate
    while (!m_armSubsystem.isCalibrated()) {
      m_armSubsystem.calibrate();
    }

    m_autoTimer.start();

    if (m_autoTimer.get() < 1) {
      m_armSubsystem.operateArm(-.7, false, 0, 180);//first value neg up pos down
    } else if (m_autoTimer.get() < 3) {
      m_armSubsystem.operateArm(-7, false, 0, -1);
    } else if (m_autoTimer.get() < 8) {
      m_armSubsystem.operateArm(0, false, 0, -1);
      m_driveSubsystem.drive(0.5,0,false);
    } else if (m_autoTimer.get() < 11.25) { 
      m_driveSubsystem.drive(-0.5, 0, false);
    } else {
      m_driveSubsystem.drive(0,0,false);
    }
 
  // Called once the command ends or is interrupted.
  }

  @Override
  public void end(boolean interrupted) {
    m_armSubsystem.operateArm(0, false, 0, -1);
    m_autoTimer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
