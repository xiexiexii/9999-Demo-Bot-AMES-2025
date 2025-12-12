package frc.robot.commands.Superstructure;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Superstructure.ShooterSubsystem;
import frc.robot.subsystems.Superstructure.TowerSubsystem;

public class PrimeTowerCommand extends Command{
   
    // Instantiate stuff
    TowerSubsystem m_towerSubsystem;
    ShooterSubsystem m_shooterSubsystem;

    Timer m_timer = new Timer();
    double m_seconds = 0.4;

    public PrimeTowerCommand(TowerSubsystem towerSubsystem, ShooterSubsystem shooterSubsystem) {

      // Definitions and setting parameters equal to members
      m_towerSubsystem = towerSubsystem;
      m_shooterSubsystem = shooterSubsystem;

      addRequirements(m_towerSubsystem);
      addRequirements(m_shooterSubsystem);
    }


    public void initialize() {
      m_timer.start();
      m_timer.reset();
    }
        

    public void execute() {
      m_towerSubsystem.stopFeeder();
      m_towerSubsystem.reverseIndexer();
      m_shooterSubsystem.reverseShooterIndex();
    }

    public void end(boolean interrupted){
      m_towerSubsystem.stopIndexer();
      m_shooterSubsystem.stopShooter();
    }

    public boolean isFinished(){
      return m_timer.hasElapsed(m_seconds);
    }

}