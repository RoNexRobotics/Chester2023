// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AutoOneCmd;
import frc.robot.commands.CalibrateArmCmd;
import frc.robot.commands.OpenLowerSolenoidCmd;
import frc.robot.commands.OpenUpperSolenoidCmd;
import frc.robot.commands.ResetDriveEncodersCmd;
import frc.robot.commands.RunVacuumCmd;
import frc.robot.commands.TeleopArmCmd;
import frc.robot.commands.TeleopDriveCmd;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.TankDriveSubsystem;
import frc.robot.subsystems.VacuumSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // Controllers
  private final Joystick m_driverController = new Joystick(OperatorConstants.kDriverControllerPort);
  private final XboxController m_operatorController = new XboxController(OperatorConstants.kArmControllerPort);

  // Subsystems
  private final TankDriveSubsystem m_driveSubsystem = new TankDriveSubsystem();
  private final ArmSubsystem m_armSubsystem = new ArmSubsystem();
  private final VacuumSubsystem m_vacuumSubsystem = new VacuumSubsystem();

  // Commands
  private final TeleopDriveCmd m_teleopDriveCmd = new TeleopDriveCmd(m_driveSubsystem, m_driverController);
  private final ResetDriveEncodersCmd m_resetDriveEncodersCmd = new ResetDriveEncodersCmd(m_driveSubsystem);
  private final TeleopArmCmd m_teleopArmCmd = new TeleopArmCmd(m_armSubsystem, m_operatorController);
  private final CalibrateArmCmd m_calibrateArmCmd = new CalibrateArmCmd(m_armSubsystem);
  private final RunVacuumCmd m_runVacuumCmd = new RunVacuumCmd(m_vacuumSubsystem);
  private final OpenUpperSolenoidCmd m_openUpperSolenoidCmd = new OpenUpperSolenoidCmd(m_vacuumSubsystem);
  private final OpenLowerSolenoidCmd m_openLowerSolenoidCmd = new OpenLowerSolenoidCmd(m_vacuumSubsystem);
  private final AutoOneCmd m_autoOneCmd = new AutoOneCmd(m_armSubsystem, m_driveSubsystem);

  // Shuffleboard
  private final SendableChooser<Command> m_autoChooser = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    CameraServer.startAutomaticCapture();

    m_driveSubsystem.setDefaultCommand(m_teleopDriveCmd);

    m_armSubsystem.setDefaultCommand(m_teleopArmCmd);

    m_vacuumSubsystem.setDefaultCommand(
        new InstantCommand(
            () -> m_vacuumSubsystem.vacuumOff(),
            m_vacuumSubsystem).withName("Off"));

    m_autoChooser.setDefaultOption("Auto 1", m_autoOneCmd);
    m_autoChooser.addOption("No Auto", null);

    SmartDashboard.putData("Auto Chooser", m_autoChooser);
    SmartDashboard.putData("Drive Subsystem", m_driveSubsystem);
    SmartDashboard.putData("Arm Subsystem", m_armSubsystem);
    SmartDashboard.putData("Vacuum Subsystem", m_vacuumSubsystem);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Reset drive encoders
    new JoystickButton(m_driverController, 6).whileTrue(m_resetDriveEncodersCmd);

    // Calibrate arm
    new JoystickButton(m_operatorController, XboxController.Button.kX.value).whileTrue(m_calibrateArmCmd);

    // Run vacuum
    new JoystickButton(m_operatorController, XboxController.Button.kRightBumper.value).whileTrue(m_runVacuumCmd);

    // Open upper solenoid
    new JoystickButton(m_operatorController, XboxController.Button.kA.value).whileTrue(m_openUpperSolenoidCmd);

    // Open lower solenoid
    new JoystickButton(m_operatorController, XboxController.Button.kY.value).whileTrue(m_openLowerSolenoidCmd);
  }

  public Command getAutonomousCommand() {
    return m_autoChooser.getSelected();
  }

}
