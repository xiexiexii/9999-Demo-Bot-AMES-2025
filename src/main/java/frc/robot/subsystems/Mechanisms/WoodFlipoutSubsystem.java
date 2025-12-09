package frc.robot.subsystems.Mechanisms;

import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs.TalonFXConfigs;
import frc.robot.Constants.MotorIDConstants;

public class WoodFlipoutSubsystem extends SubsystemBase{

  private TalonFX m_woodFlipout;

  public WoodFlipoutSubsystem() {
    
    m_woodFlipout = new TalonFX(MotorIDConstants.k_woodFlipoutMotorID);

    m_woodFlipout.getConfigurator().apply(TalonFXConfigs.woodFlipoutConfig, 0.05);
  }

  public Angle getWristPosition(){
    return Units.Rotations.of(m_woodFlipout.get());
  }

  public void setPosition(Angle angle){
    m_woodFlipout.setControl(new PositionVoltage(angle.in(Units.Rotations)));
  }

  public void setNeutral() {
    m_woodFlipout.setControl(new NeutralOut());
  }

  public void resetSensorPosition(Angle setpoint) {
    m_woodFlipout.setPosition(setpoint.in(Units.Rotations));
  }

  public double getCurrentPosition() {
    return m_woodFlipout.getPosition().getValueAsDouble();
  }

  public double getCurrentVelocity() {
    return m_woodFlipout.getVelocity().getValueAsDouble();
  }

  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Wrist/Pos", Units.Rotations.of(m_woodFlipout.getPosition().getValueAsDouble()).magnitude());
    
    /*
    SmartDashboard.putString("Wrist/Units", m_woodFlipout.getPosition().getUnits());
    SmartDashboard.putNumber("Wrist/CLO", m_woodFlipout.getClosedLoopOutput().getValueAsDouble());
    SmartDashboard.putNumber("Wrist/Output", m_woodFlipout.get());
    SmartDashboard.putNumber("Wrist/Inverted", m_woodFlipout.getAppliedRotorPolarity().getValueAsDouble());
    SmartDashboard.putNumber("Wrist/Current", m_woodFlipout.getSupplyCurrent().getValueAsDouble());
    */
  }
}
