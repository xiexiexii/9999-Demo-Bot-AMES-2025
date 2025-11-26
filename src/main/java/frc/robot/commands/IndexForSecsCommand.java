package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Mechanisms.ShooterSubsystem;
import frc.robot.subsystems.Mechanisms.TowerSubsystem;

public class IndexForSecsCommand extends Command{
   
    // Instantiate stuff
    TowerSubsystem m_towerSubsystem;
    ShooterSubsystem m_shooterSubsystem;

    Timer m_timer = new Timer();
    double m_seconds;

    public IndexForSecsCommand(TowerSubsystem towerSubsystem, ShooterSubsystem shooterSubsystem, double seconds) {

      // Definitions and setting parameters equal to members
      m_towerSubsystem = towerSubsystem;
      m_shooterSubsystem = shooterSubsystem;
      m_seconds = seconds;

      addRequirements(m_towerSubsystem);
      addRequirements(m_shooterSubsystem);
    }


    public void initialize() {
      m_timer.start();
      m_timer.reset();
    }
        

    public void execute() {
      m_towerSubsystem.feed();
      m_towerSubsystem.index();
      m_shooterSubsystem.reverseShooterIndex();
    }

    public void end(boolean interrupted){
      m_towerSubsystem.stopFeeder();
      m_towerSubsystem.stopIndexer();
      m_shooterSubsystem.stopShooter();
    }

    public boolean isFinished(){
      return m_timer.hasElapsed(m_seconds);
    }

}