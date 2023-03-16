// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VacuumSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // Robot subsystems
  private DriveSubsystem m_driveSubsystem = new DriveSubsystem();
  private VacuumSubsystem m_vacuumSubsystem = new VacuumSubsystem();

  // Controllers
  private Joystick m_driverController = new Joystick(OperatorConstants.kDriverControllerPort);
  private XboxController m_armController = new XboxController(OperatorConstants.kArmControllerPort);

  // Network Table Instance
  private NetworkTableInstance m_netInst;
  private NetworkTable m_limelight;
  private NetworkTableEntry m_limelightX;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_netInst = NetworkTableInstance.getDefault();
    m_limelight = m_netInst.getTable("limelight");
    m_limelightX = m_limelight.getEntry("tx");

    // Configure the trigger bindings
    configureBindings();

    m_driveSubsystem.setDefaultCommand(
      new RunCommand(
        () -> m_driveSubsystem.drive(
          -m_driverController.getY() * DriveConstants.kPowerPercent,
          -m_driverController.getZ() * DriveConstants.kAngularPowerPercent,
          m_driverController.getPOV()
        ),
        m_driveSubsystem).withName("Joystick Drive")
    );

    m_vacuumSubsystem.setDefaultCommand(
      new InstantCommand(
        () -> m_vacuumSubsystem.off(),
        m_vacuumSubsystem).withName("Off")
    );

    SmartDashboard.putData(m_driveSubsystem);
    SmartDashboard.putData(m_vacuumSubsystem);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Vacuum forward
    new JoystickButton(m_armController, XboxController.Button.kRightBumper.value).onTrue(
      new RunCommand(
        () -> m_vacuumSubsystem.on(),
        m_vacuumSubsystem).withName("On")
    ).onFalse(
      new RunCommand(
        () -> m_vacuumSubsystem.off(),
        m_vacuumSubsystem).withName("Off")
    );

    // Vacuum reversed
    new JoystickButton(m_armController, XboxController.Button.kLeftBumper.value).onTrue(
      new RunCommand(
        () -> m_vacuumSubsystem.onInReverse(),
        m_vacuumSubsystem).withName("On In Reverse")
    ).onFalse(
      new RunCommand(
        () -> m_vacuumSubsystem.off(),
        m_vacuumSubsystem).withName("Off")
    );

    // Reset drive encoders
    new JoystickButton(m_driverController, 6).onTrue(
      new InstantCommand(
        () -> m_driveSubsystem.alignModulesEnabled(false)
      )
    ).onFalse(
      new InstantCommand(
        () -> m_driveSubsystem.alignModulesEnabled(true)
      ).andThen(
        () -> m_driveSubsystem.resetEncoders()
      )
    );
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Use RunCommand for repeating code and InstantCommand for one-time code
    // Use Thread.sleep(milliseconds) surrounded by a try-catch statement to delay the autonomous code
    return new RunCommand(
      () -> {
        SmartDashboard.putNumber("Limelight X", m_limelightX.getDouble(0));

        double limelightX = m_limelightX.getDouble(0);
        
        if (limelightX <= -10) { // Left fast-rotation zone
          m_driveSubsystem.drive(0.4, 0.7);
        } else if (limelightX >= 10) { // Right fast-rotation zone
          m_driveSubsystem.drive(0.4, -0.7);
        } else if (limelightX > -10 && limelightX < -1) { // Left slow-rotation zone
          m_driveSubsystem.drive(0.5, 0.4);
        } else if (limelightX < 10 && limelightX > 1) { // Right slow-rotation zone
          m_driveSubsystem.drive(0.5, -0.4);
        } else { // Target achieved
          m_driveSubsystem.drive(0.5, 0);
        }
      },
      m_driveSubsystem).withName("Auto");
  }
}
