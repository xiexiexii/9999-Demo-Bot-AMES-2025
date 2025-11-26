// Imports stuff (again!)

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.PrimeTowerCommand;
import frc.robot.commands.SmartIntakeCommand;
import frc.robot.commands.SpinUpShootCommand;
import frc.robot.subsystems.Mechanisms.CANRangeSubsystem;
import frc.robot.subsystems.Mechanisms.IntakeSubsystem;
import frc.robot.subsystems.Mechanisms.ShooterSubsystem;
import frc.robot.subsystems.Mechanisms.TowerSubsystem;
import frc.robot.subsystems.Swerve.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

// This class is where the bulk of the robot should be declared.  Since Command-based is a
// "declarative" paradigm, very little robot logic should actually be handled in the Robot
// periodic methods (other than the scheduler calls).  Instead, the structure of the robot
// (including subsystems, commands, and button mappings) should be declared here.

public class RobotContainer {

  // Robot's Subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  private final TowerSubsystem m_towerSubsystem = new TowerSubsystem();
  private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
  private final CANRangeSubsystem m_CANRangeSubsystem = new CANRangeSubsystem();

  // Chooser for Auto
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  // Controllers
  XboxController m_driverController = new XboxController(ControllerConstants.kDriverControllerPort);

  // The container for the robot. Contains subsystems, OI devices, and commands.
  public RobotContainer() {

    // Configure the button bindings
    configureButtonBindings();

    // Configure Default Commands
    m_robotDrive.setDefaultCommand(
      
      // Translation of the robot is controlled by the left stick
      // Turning is controlled by the X axis of the right stick
      new RunCommand(
        () -> m_robotDrive.drive(
          -MathUtil.applyDeadband(m_driverController.getLeftY(), ControllerConstants.kDriveDeadband),
          -MathUtil.applyDeadband(m_driverController.getLeftX(), ControllerConstants.kDriveDeadband),
          -MathUtil.applyDeadband(m_driverController.getRightX(), ControllerConstants.kDriveDeadband),
          true),
        m_robotDrive
      )
    );

    // Chooser window on SmartDashboard/Shuffleboard/Elastic
    SmartDashboard.putData("AutoMode", m_chooser);

    // Named Command Configuration
    // NamedCommands.registerCommand("Change LED Color", new LEDColorChangeCommand(m_LEDSubsystem));

    // Autos
    m_chooser.addOption("Curvy yay", m_robotDrive.getAuto("Curvy yay"));
  }

  // Define Button and Axis bindings here
  private void configureButtonBindings() {

    // Sets wheels in an X position to prevent movement - A
    new JoystickButton(m_driverController, ControllerConstants.k_A)
      .whileTrue(new RunCommand(() -> m_robotDrive.setX(), m_robotDrive)
    );

    // Zero Gyro - Back
    new JoystickButton(m_driverController, ControllerConstants.k_back)
      .onTrue(new InstantCommand(() -> m_robotDrive.zeroHeading(), m_robotDrive)
    );

    // Intake - Right Trig
    new Trigger(() -> m_driverController.getRawAxis(ControllerConstants.k_righttrig) > 0.05)
      .onTrue(
        new SmartIntakeCommand(m_intakeSubsystem, m_towerSubsystem, m_shooterSubsystem, m_CANRangeSubsystem)
      )
      .onFalse(
        new PrimeTowerCommand(m_towerSubsystem, m_shooterSubsystem).alongWith(
        new InstantCommand(() -> m_intakeSubsystem.stopIntake(), m_intakeSubsystem)
      )
    );

    // Index - Right Bump
    new JoystickButton(m_driverController, ControllerConstants.k_rightbump)
      .whileTrue(
        new InstantCommand(() -> m_towerSubsystem.indexAll(), m_towerSubsystem)
      )
      .onFalse(
        new PrimeTowerCommand(m_towerSubsystem, m_shooterSubsystem)
    );

    // Shoot - Left Trig
    new Trigger(() -> m_driverController.getRawAxis(ControllerConstants.k_lefttrig) > 0.05)
      .onTrue(
        new SpinUpShootCommand(m_towerSubsystem, m_shooterSubsystem)
      )
      .onFalse(
        stopShooterCommand()
    );
  }

  public Command getAutonomousCommand() {
    return m_chooser.getSelected();   
  } 

  public ParallelCommandGroup stopShooterCommand() {
    return new ParallelCommandGroup(
      new InstantCommand(() -> m_shooterSubsystem.stopShooter(), m_shooterSubsystem),
      new InstantCommand(() -> m_towerSubsystem.stopAll(), m_towerSubsystem)
    );
  }

  public ParallelCommandGroup intakeParallelCommand() {
    return new ParallelCommandGroup(
      new InstantCommand(() -> m_intakeSubsystem.intake(), m_intakeSubsystem),
      new InstantCommand(() -> m_shooterSubsystem.reverseShooterIndex(), m_shooterSubsystem),
      new InstantCommand(() -> m_towerSubsystem.indexAll(), m_towerSubsystem)
    );
  }

  public ParallelCommandGroup stopIntakeParallelCommand() {
    return new ParallelCommandGroup(
      new InstantCommand(() -> m_intakeSubsystem.stopIntake(), m_intakeSubsystem),
      new InstantCommand(() -> m_shooterSubsystem.stopShooter(), m_shooterSubsystem),
      new InstantCommand(() -> m_towerSubsystem.stopAll(), m_towerSubsystem)
    );
  }
}