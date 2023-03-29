package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VacuumConstants;

public class VacuumSubsystem extends SubsystemBase {
  private VictorSPX m_vacuumMotor;
  private Solenoid m_upperSolenoid;
  private Solenoid m_lowerSolenoid;

  public VacuumSubsystem() {
    m_vacuumMotor = new VictorSPX(VacuumConstants.kVacuumMotorId);
    m_upperSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, VacuumConstants.kUpperSolenoidId);
    m_lowerSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, VacuumConstants.kLowerSolenoidId);
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Upper Solenoid", m_upperSolenoid.get());
    SmartDashboard.putBoolean("Lower Solenoid", m_lowerSolenoid.get());
  }

  public void vacuumOn() {
    m_vacuumMotor.set(ControlMode.PercentOutput, 1);
  }

  public void vacuumOff() {
    m_vacuumMotor.set(ControlMode.PercentOutput, 0);
  }

  public void openUpperSolenoid() {
    m_upperSolenoid.set(true);
  }

  public void closeUpperSolenoid() {
    m_upperSolenoid.set(false);
  }

  public void openLowerSolenoid() {
    m_lowerSolenoid.set(true);
  }

  public void closeLowerSolenoid() {
    m_lowerSolenoid.set(false);
  }
}