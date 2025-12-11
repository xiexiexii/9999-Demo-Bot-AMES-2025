package frc.robot.subsystems.Mechanisms;

import static edu.wpi.first.units.Units.Inches;

import com.ctre.phoenix6.hardware.CANrange;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs.SensorConfigs;
import frc.robot.Constants.LiveConstants;
import frc.robot.Constants.SensorIDConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Limelight.LimelightHelpers;

public class CANRangeSubsystem extends SubsystemBase{
  
  private CANrange m_canrange;

  public CANRangeSubsystem() {
    m_canrange = new CANrange(SensorIDConstants.k_intakeCANRangeID);
    m_canrange.getConfigurator().apply(SensorConfigs.CANRangeConfig, 0.05);
  }

  public boolean getIsDetected() {
    return m_canrange.getDistance().getValue().isNear(Inches.of(1.5), 1.5);
}

  public void periodic() {
    SmartDashboard.putBoolean("canrange", getIsDetected());
    SmartDashboard.putNumber("canrange/intakeCounter", LiveConstants._intakeCounter);

    SmartDashboard.putNumber("Limelight-Intake/Intake TX", LimelightHelpers.getTX(VisionConstants.k_intakeLimelightName));
    SmartDashboard.putNumber("Limelight-Intake/Intake TY", LimelightHelpers.getTY(VisionConstants.k_intakeLimelightName));
    SmartDashboard.putBoolean("Limelight-Intake/Intake TV", LimelightHelpers.getTV(VisionConstants.k_intakeLimelightName));
    SmartDashboard.putString("Limelight-Intake/Class", LimelightHelpers.getDetectorClass(VisionConstants.k_intakeLimelightName));

    SmartDashboard.putNumber("Limelight-Shoot/Shoot TX", LimelightHelpers.getTX(VisionConstants.k_shooterLimelightName));
    SmartDashboard.putNumber("Limelight-Shoot/Shoot TY", LimelightHelpers.getTY(VisionConstants.k_shooterLimelightName));
    SmartDashboard.putBoolean("Limelight-Shoot/Shoot TV", LimelightHelpers.getTV(VisionConstants.k_shooterLimelightName));
  }
}