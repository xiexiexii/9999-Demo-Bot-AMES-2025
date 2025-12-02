package frc.robot.subsystems.Mechanisms;

import com.ctre.phoenix6.hardware.TalonFXS;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs.MinionConfigs;
import frc.robot.Constants.MotorIDConstants;
import frc.robot.Constants.MotorSpeedConstants;

public class IntakeSubsystem extends SubsystemBase{
  
  // Motors - [CTRE Minion + Talon FXS] x1
  TalonFXS m_intake;

  public IntakeSubsystem() {
    
    m_intake = new TalonFXS(MotorIDConstants.k_intakeMotorID);

    m_intake.getConfigurator().apply(MinionConfigs.intakeConfig, 0.05);
  }

  // Intake Stuff
  public void intake() {
    m_intake.set(MotorSpeedConstants.k_intakeSpeed);
  }

  public void stopIntake() {
    m_intake.set(0);
  }

  // Just in case 
  public void reverseIntake() {
    m_intake.set(-MotorSpeedConstants.k_intakeSpeed);
  }

  public void periodic() {}
}
