package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VacuumConstants;

public class VacuumSubsystem extends SubsystemBase {
  VictorSPX m_vacuumMotor = new VictorSPX(20);

  public VacuumSubsystem() {
    m_vacuumMotor = new VictorSPX(VacuumConstants.kVacuumMotorId);
  }

  public void on() {
    m_vacuumMotor.set(ControlMode.PercentOutput, 1);
  }

  public void off() {
    m_vacuumMotor.set(ControlMode.PercentOutput, 1);
  }
}
