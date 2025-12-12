package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.TimeConstants;
import frc.robot.subsystems.Superstructure.ShooterSubsystem;
import frc.robot.subsystems.Superstructure.TowerSubsystem;

public class SpinUpShootAutoCommand extends Command{
   
    // Instantiate stuff
    TowerSubsystem m_towerSubsystem;
    ShooterSubsystem m_shooterSubsystem;

    Timer m_timer = new Timer();
    double m_spinUpTime = TimeConstants.k_spinUpTime;
    double m_shootTime = TimeConstants.k_shootTime;

    public SpinUpShootAutoCommand(TowerSubsystem towerSubsystem, ShooterSubsystem shooterSubsystem) {

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
      m_shooterSubsystem.shoot();

      if(m_timer.hasElapsed(m_spinUpTime)) {
        m_towerSubsystem.indexAll();
      }
    }

    public void end(boolean interrupted){
      m_shooterSubsystem.stopShooter();
      m_towerSubsystem.stopAll();
    }

    public boolean isFinished(){
      return m_timer.hasElapsed(m_shootTime);
    }

}