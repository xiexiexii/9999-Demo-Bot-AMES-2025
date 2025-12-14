// Imports stuff (again!)

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.WoodFlipout.IntakeWoodForSecsAutoCommand;
import frc.robot.commands.WoodFlipout.IntakeWoodForSecsCommand;
import frc.robot.commands.WoodFlipout.SetWoodFlipoutCommand;
import frc.robot.commands.WoodFlipout.ShootWoodCommand;
import frc.robot.commands.WoodFlipout.ZeroWoodFlipoutCommand;
import frc.robot.commands.Limelight.IntakeAutoPositionCommand;
import frc.robot.commands.Limelight.ShootAutoPositionCommand;
import frc.robot.commands.Shooter.SpinUpShootAutoCommand;
import frc.robot.commands.Shooter.SpinUpShootCommand;
import frc.robot.commands.Shooter.SpinUpSnipeAutoCommand;
import frc.robot.commands.Superstructure.PrimeTowerCommand;
import frc.robot.commands.Superstructure.SmartIntakeCommand;
import frc.robot.subsystems.Superstructure.CANRangeSubsystem;
import frc.robot.subsystems.Superstructure.ShooterSubsystem;
import frc.robot.subsystems.Superstructure.TowerSubsystem;
import frc.robot.subsystems.Swerve.DriveSubsystem;
import frc.robot.subsystems.VegetableFlipout.FlipTakeSubsystem;
import frc.robot.subsystems.VegetableFlipout.IntakeSubsystem;
import frc.robot.subsystems.WoodFlipout.WoodFlipoutSubsystem;
import frc.robot.subsystems.WoodFlipout.WoodIntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
  private final FlipTakeSubsystem m_flipTakeSubsystem = new FlipTakeSubsystem();
  private final WoodIntakeSubsystem m_woodIntakeSubsystem = new WoodIntakeSubsystem();
  private final WoodFlipoutSubsystem m_woodFlipoutSubsystem = new WoodFlipoutSubsystem();

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
    NamedCommands.registerCommand("Shoot Vegetables", new SpinUpShootAutoCommand(m_towerSubsystem, m_shooterSubsystem));
    NamedCommands.registerCommand("Snipe Vegetables", new SpinUpSnipeAutoCommand(m_towerSubsystem, m_shooterSubsystem));
    NamedCommands.registerCommand("Score Wood", scoreWoodCommand());

    // Trigger Configuration
    // new EventTrigger("Extend Wood Flipout").onTrue(new InstantCommand(() -> m_woodFlipoutSubsystem.setPosition(WoodFlipoutConstants.k_woodFlipoutIntakeAngle), m_woodFlipoutSubsystem));
    // new EventTrigger("Retract Wood Flipout").onTrue(finishWoodIntakeCommand());
    // new EventTrigger("Intake Wood").onTrue(new IntakeWoodForSecsCommand(m_woodIntakeSubsystem, 3));

    NamedCommands.registerCommand("Extend Wood Flipout", new SetWoodFlipoutCommand(m_woodFlipoutSubsystem, "INTAKE"));
    // NamedCommands.registerCommand("Retract Wood Flipout", finishWoodIntakeCommand());
    NamedCommands.registerCommand("Intake Wood", intakeWoodAutoCommand());

    // Autos TODO: DS always blue?
    m_chooser.addOption("[40 pts] 3V-2W Air Fryer Supercarry", m_robotDrive.getAuto("[3V 2W] Firewood Supercarry"));
    m_chooser.addOption("[28 pts] 3V-0W Cabin Park RED", m_robotDrive.getAuto("[3V] Cabin Park RED"));
    m_chooser.addOption("[28 pts] 3V-0W Cabin Park BLUE", m_robotDrive.getAuto("[3V] Cabin Park BLUE"));
    m_chooser.addOption("[32 pts] 3V-1W Leave", m_robotDrive.getAuto("[3V 1W] Charged Leave"));
    m_chooser.addOption("[2 pts] Leave", m_robotDrive.getAuto("Leave"));
  }

  // Define Button and Axis bindings here
  private void configureButtonBindings() {

    // Sets wheels in an X position to prevent movement - A
    new JoystickButton(m_driverController, ControllerConstants.k_A)
      .whileTrue(new RunCommand(() -> m_robotDrive.setX(), m_robotDrive)
    );

    // Auto-Intake - B
    new JoystickButton(m_driverController, ControllerConstants.k_B)
      .onTrue(new IntakeAutoPositionCommand(m_robotDrive).alongWith(
        new SmartIntakeCommand(m_intakeSubsystem, m_towerSubsystem, m_shooterSubsystem, m_CANRangeSubsystem))
      )
      .onFalse(
        new PrimeTowerCommand(m_towerSubsystem, m_shooterSubsystem).alongWith(
        new InstantCommand(() -> m_intakeSubsystem.stopIntake(), m_intakeSubsystem)
      )
    );

    // Zero Wood Flipout - X
    new JoystickButton(m_driverController, ControllerConstants.k_X)
      .onTrue(
        new ZeroWoodFlipoutCommand(m_woodFlipoutSubsystem)
    );

    // Zero Gyro - Back
    new JoystickButton(m_driverController, ControllerConstants.k_back)
      .onTrue(new InstantCommand(() -> m_robotDrive.zeroHeading(), m_robotDrive)
    );

    // Intake - Right Bump
    new JoystickButton(m_driverController, ControllerConstants.k_rightbump)
      .onTrue(
        intakeParallelCommand().alongWith(
        resetWoodFlipoutCommand())
      )
      .onFalse(
        stopIntakeParallelCommand()
    );

    // Intake Wood - Right Trig
    new Trigger(() -> m_driverController.getRawAxis(ControllerConstants.k_righttrig) > 0.50)
      .onTrue(
        new SetWoodFlipoutCommand(m_woodFlipoutSubsystem, "INTAKE").alongWith(
        new InstantCommand(() -> m_flipTakeSubsystem.retract(), m_flipTakeSubsystem))
      )
      .whileTrue(
        new InstantCommand(() -> m_woodIntakeSubsystem.intake(), m_woodIntakeSubsystem)
      )
      .onFalse(
        finishWoodIntakeCommand()
    );

    // Limelight Shoot - Left Bump
    new JoystickButton(m_driverController, ControllerConstants.k_leftbump) 
      .onTrue(
        shootVegetableCommandGroup()
      )
      .onFalse(
        stopShooterCommand()
    );

    // Shoot Wood - Left Trig
    new Trigger(() -> m_driverController.getRawAxis(ControllerConstants.k_lefttrig) > 0.50)
      .onTrue(
        scoreWoodCommand()
      )
      .onFalse(
        resetWoodFlipoutCommand()
      );
  }

  // Returns the command to run in autonomous
  public Command getAutonomousCommand() {
    return m_chooser.getSelected();   
  } 

  // COMMAND GROUPS ARE BELOW

  // Shoots Vegetables
  public SequentialCommandGroup shootVegetableCommandGroup() {
    return new SequentialCommandGroup(
      new ParallelCommandGroup(
        new SetWoodFlipoutCommand(m_woodFlipoutSubsystem, "LIMELIGHT"),
        new IntakeWoodForSecsCommand(m_woodIntakeSubsystem, 0.5)
      ),
      new ParallelCommandGroup(
        new ShootAutoPositionCommand(m_robotDrive),
        new SpinUpShootCommand(m_towerSubsystem, m_shooterSubsystem)
      ),
      new SetWoodFlipoutCommand(m_woodFlipoutSubsystem, "RESET"),
      new ZeroWoodFlipoutCommand(m_woodFlipoutSubsystem)
    );
  }

  // Stops Shooter
  public ParallelCommandGroup stopShooterCommand() {
    return new ParallelCommandGroup(
      new InstantCommand(() -> m_shooterSubsystem.stopShooter(), m_shooterSubsystem),
      new InstantCommand(() -> m_towerSubsystem.stopAll(), m_towerSubsystem)
    );
  }

  // Intake Vegetables
  public ParallelCommandGroup intakeParallelCommand() {
    return new ParallelCommandGroup(
      new InstantCommand(() -> m_flipTakeSubsystem.extend(), m_flipTakeSubsystem),
      new SmartIntakeCommand(m_intakeSubsystem, m_towerSubsystem, m_shooterSubsystem, m_CANRangeSubsystem)
    );
  }

  // Stop Running Vegetable Intake
  public ParallelCommandGroup stopIntakeParallelCommand() {
    return new ParallelCommandGroup(
      // new InstantCommand(() -> m_flipTakeSubsystem.retract(), m_flipTakeSubsystem),
      new PrimeTowerCommand(m_towerSubsystem, m_shooterSubsystem),
      new InstantCommand(() -> m_intakeSubsystem.stopIntake(), m_intakeSubsystem)
    );
  }

  // Reset Wood Flipout
  public SequentialCommandGroup resetWoodFlipoutCommand() {
    return new SequentialCommandGroup(
      new SetWoodFlipoutCommand(m_woodFlipoutSubsystem, "RESET"),
      new ZeroWoodFlipoutCommand(m_woodFlipoutSubsystem)
    );
  }

  // Finish Wood Intake Command
  public ParallelDeadlineGroup finishWoodIntakeCommand() {
    return new ParallelDeadlineGroup(
      new SequentialCommandGroup(
        new SetWoodFlipoutCommand(m_woodFlipoutSubsystem, "RESET"),
        new ZeroWoodFlipoutCommand(m_woodFlipoutSubsystem)
      ),
      new IntakeWoodForSecsCommand(m_woodIntakeSubsystem, 0.7)
    );
  }

  // Score Wood
  public SequentialCommandGroup scoreWoodCommand() {
    return new SequentialCommandGroup(
      new ParallelCommandGroup(
        new SetWoodFlipoutCommand(m_woodFlipoutSubsystem, "SCORE"),
        new InstantCommand(() -> m_flipTakeSubsystem.retract(), m_flipTakeSubsystem),
        new ShootWoodCommand(m_woodIntakeSubsystem, 1.2)
      ),
      new SetWoodFlipoutCommand(m_woodFlipoutSubsystem, "RESET"),
      new ZeroWoodFlipoutCommand(m_woodFlipoutSubsystem),
      Commands.print("SCORE DONE!")
    );
  }

  // Intake Wood in Auto
  public SequentialCommandGroup intakeWoodAutoCommand() {
    return new SequentialCommandGroup(
      new IntakeWoodForSecsAutoCommand(m_woodIntakeSubsystem, 1.1),
      new SetWoodFlipoutCommand(m_woodFlipoutSubsystem, "RESET"),
      new ZeroWoodFlipoutCommand(m_woodFlipoutSubsystem),
      Commands.print("INTAKE DONE!")
    );
  }
}