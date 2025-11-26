package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LiveConstants;
import frc.robot.subsystems.Mechanisms.CANRangeSubsystem;
import frc.robot.subsystems.Mechanisms.IntakeSubsystem;
import frc.robot.subsystems.Mechanisms.ShooterSubsystem;
import frc.robot.subsystems.Mechanisms.TowerSubsystem;

public class SmartIntakeCommand extends Command{
   
    // Instantiate stuff
    IntakeSubsystem m_intakeSubsystem;
    TowerSubsystem m_towerSubsystem;
    ShooterSubsystem m_shooterSubsystem;
    CANRangeSubsystem m_CANRangeSubsystem;

    Timer m_timer = new Timer();
    Boolean m_isFinished;

    public SmartIntakeCommand(IntakeSubsystem intakeSubsystem, TowerSubsystem towerSubsystem, ShooterSubsystem shooterSubsystem, CANRangeSubsystem canrangeSubsystem) {

      // Definitions and setting parameters equal to members
      m_intakeSubsystem = intakeSubsystem;
      m_towerSubsystem = towerSubsystem;
      m_shooterSubsystem = shooterSubsystem;
      m_CANRangeSubsystem = canrangeSubsystem;

      addRequirements(m_intakeSubsystem);
      addRequirements(m_towerSubsystem);
      addRequirements(m_shooterSubsystem);
      addRequirements(m_CANRangeSubsystem);
    }


    public void initialize() {
      m_timer.start();
      m_timer.reset();

      m_isFinished = false;
    }
        

    public void execute() {
      m_intakeSubsystem.intake();
      m_towerSubsystem.feed();
      m_towerSubsystem.index();
      m_shooterSubsystem.reverseShooterIndex();

      if (m_CANRangeSubsystem.getIsDetected() == true && LiveConstants._lastIsDetected == false) LiveConstants._intakeCounter += 1;
      if (m_CANRangeSubsystem.getIsDetected() && (LiveConstants._intakeCounter % 3) == 0) m_isFinished = true;

      LiveConstants._lastIsDetected = m_CANRangeSubsystem.getIsDetected();
    }

    public void end(boolean interrupted){
      m_intakeSubsystem.stopIntake();
      m_towerSubsystem.stopFeeder();
      m_towerSubsystem.stopIndexer();
      m_shooterSubsystem.stopShooter();
    }

    public boolean isFinished(){
      return m_isFinished;
    }

}