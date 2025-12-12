package frc.robot.subsystems.VegetableFlipout;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PneumaticConstants;

public class FlipTakeSubsystem extends SubsystemBase{

  // Compressor [Piston + Double Solenoid] x2
  Compressor m_compressor;
  DoubleSolenoid m_flipSolenoidLeft;
  DoubleSolenoid m_flipSolenoidRight;
  
  public FlipTakeSubsystem() {

    // Compressor should start automatically (I think)
    m_compressor = new Compressor(PneumaticsModuleType.REVPH); 

    // Set Solenoid Ports
    m_flipSolenoidLeft = new DoubleSolenoid(
      PneumaticsModuleType.REVPH,
      PneumaticConstants.k_solenoidIDFlipTakeLeftForwardChannel,
      PneumaticConstants.k_solenoidIDFlipTakeLeftReverseChannel
    );

    m_flipSolenoidRight = new DoubleSolenoid(
      PneumaticsModuleType.REVPH,
      PneumaticConstants.k_solenoidIDFlipTakeRightForwardChannel,
      PneumaticConstants.k_solenoidIDFlipTakeRightReverseChannel
    );

    // Set Solenoid Initial State (Retracted)
    m_flipSolenoidLeft.set(DoubleSolenoid.Value.kReverse);
    m_flipSolenoidRight.set(DoubleSolenoid.Value.kReverse);
  }

  // Extend Pistons
  public void extend() {

    // Extend!
    m_flipSolenoidLeft.set(DoubleSolenoid.Value.kForward);
    m_flipSolenoidRight.set(DoubleSolenoid.Value.kForward);
  }

  // Extend Individual Pistons
  public void extendLeftOnly() {
    m_flipSolenoidLeft.set(DoubleSolenoid.Value.kForward);
  }

  public void extendRightOnly() {
    m_flipSolenoidRight.set(DoubleSolenoid.Value.kForward);
  }

  // Retract Pistons
  public void retract() {

    // Retract!
    m_flipSolenoidLeft.set(DoubleSolenoid.Value.kReverse);
    m_flipSolenoidRight.set(DoubleSolenoid.Value.kReverse);
  }

  // Retract Individual Pistons
  public void retractLeftOnly() {
    m_flipSolenoidLeft.set(DoubleSolenoid.Value.kReverse);
  }

  public void retractRightOnly() {
    m_flipSolenoidRight.set(DoubleSolenoid.Value.kReverse);
  }

  public void periodic() {

    SmartDashboard.putNumber("FlipTake/Compressor/Current Draw", m_compressor.getCurrent());
    SmartDashboard.putBoolean("FlipTake/Compressor/Compressor Enabled", m_compressor.isEnabled());
  }
}
