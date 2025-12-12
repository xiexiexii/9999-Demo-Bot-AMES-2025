package frc.robot;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.signals.AdvancedHallSupportValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.UpdateModeValue;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.units.Units;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.WoodFlipoutConstants;

public final class Configs {
  public static final class MAXSwerveModule {
    public static final SparkFlexConfig drivingConfig = new SparkFlexConfig();
    public static final SparkMaxConfig turningConfig = new SparkMaxConfig();

    static {
      // Use module constants to calculate conversion factors and feed forward gain.
      double drivingFactor = ModuleConstants.kWheelDiameterMeters * Math.PI
        / ModuleConstants.kDrivingMotorReduction;
      double turningFactor = 2 * Math.PI;
      double drivingVelocityFeedForward = 1 / ModuleConstants.kDriveWheelFreeSpeedRps;

      /*
       ********************************************
       **   DRIVEBASE NEO VORTEX CONFIGURATION   **
       ********************************************
      */

      drivingConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(50);
      drivingConfig.encoder
        .positionConversionFactor(drivingFactor) // meters
        .velocityConversionFactor(drivingFactor / 60.0); // meters per second
      drivingConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        // These are example gains you may need to them for your own robot!
        .pid(0.04, 0, 0)
        .velocityFF(drivingVelocityFeedForward)
        .outputRange(-1, 1);

      /*
       ********************************************
       **     DRIVEBASE NEO 550 CONFIGURATION    **
       ********************************************
      */

      turningConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(20);
      turningConfig.absoluteEncoder
        // Invert the turning encoder, since the output shaft rotates in the opposite
        // direction of the steering motor in the MAXSwerve Module.
        .inverted(true)
        .positionConversionFactor(turningFactor) // radians
        .velocityConversionFactor(turningFactor / 60.0); // radians per second
      turningConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        // These are example gains you may need to tune them for your own robot!
        .pid(1, 0, 0)
        .outputRange(-1, 1)
        // Enable PID wrap around for the turning motor. This will allow the PID
        // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
        // to 10 degrees will go through 0 rather than the other direction which is a
        // longer route.
        .positionWrappingEnabled(true)
        .positionWrappingInputRange(0, turningFactor);
    }
  }

  // Sensors 
  public static final class SensorConfigs {

    // init
    public static final CANrangeConfiguration CANRangeConfig = new CANrangeConfiguration();

    // Actual stuff to adjust
    static {

      /*
       ********************************************
       **         CANRANGE CONFIGURATION         **
       ********************************************
      */

      CANRangeConfig.FutureProofConfigs = true; // Firmware incompatability safety net

      CANRangeConfig.ProximityParams.ProximityThreshold = 0.1; // How far is far enough?
      CANRangeConfig.ProximityParams.ProximityHysteresis = 0.01; // Tolerance for measurement
      CANRangeConfig.ProximityParams.MinSignalStrengthForValidMeasurement = 2500; // Signal strength req.

      CANRangeConfig.ToFParams.UpdateFrequency = 50; // How often should it measure?
      CANRangeConfig.ToFParams.UpdateMode = UpdateModeValue.ShortRange100Hz; // Measurement mode
    }
  }

  // NEO Vortex
  public static final class VortexConfigs {

    // Init
    public static final SparkFlexConfig intakeConfig = new SparkFlexConfig();
    public static final SparkFlexConfig shooterConfig = new SparkFlexConfig();

    static {

      /*
       ********************************************
       **        NEO VORTEX CONFIGURATIONS       **
       ********************************************
      */
      
      // Intake Configs
      intakeConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(60)
        .openLoopRampRate(0.5);

      // Tower Configs
      shooterConfig
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(50)
      .openLoopRampRate(0.5);
    }
  }

  // NEO v1.1
  public static final class NeoConfigs {

    // Init
    public static final SparkMaxConfig feederConfig = new SparkMaxConfig();
    public static final SparkMaxConfig indexerConfig =  new SparkMaxConfig();

    static {

      /*
       ********************************************
       **         NEO v1.1 CONFIGURATIONS        **
       ********************************************
      */

      // Feeder Configs
      feederConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(50)
        .openLoopRampRate(0.5);

      // Shooter Configs
      indexerConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(60)
        .openLoopRampRate(0.5);
    }
  }

  // CTRE Talon FXS
  public static final class MinionConfigs {

    // Init
    public static final TalonFXSConfiguration intakeConfig = new TalonFXSConfiguration();

    static {

      /*
       ********************************************
       **      CTRE TALON FXS CONFIGURATION      **
       ********************************************
      */
      
      intakeConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0.5;
      intakeConfig.MotorOutput.PeakForwardDutyCycle = 0.65;
      intakeConfig.MotorOutput.PeakReverseDutyCycle = -0.65;
      intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      intakeConfig.CurrentLimits.SupplyCurrentLimit = 50;
      intakeConfig.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;
      intakeConfig.Commutation.AdvancedHallSupport = AdvancedHallSupportValue.Enabled;
    }
  }

  // CTRE Talon FX
  public static final class TalonFXConfigs {

    // Init
    public static final TalonFXConfiguration woodFlipoutConfig = new TalonFXConfiguration();
    public static final TalonFXConfiguration woodIntakeConfig = new TalonFXConfiguration();

    static {

      /*
       ********************************************
       **      VEX FALCON 500 CONFIGURATIONS     **
       ********************************************
      */

      // Wood Flipout Configs
      woodFlipoutConfig.Slot0.kP = WoodFlipoutConstants.k_woodFlipoutP;
      woodFlipoutConfig.Slot0.kI = WoodFlipoutConstants.k_woodFlipoutI;
      woodFlipoutConfig.Slot0.kD = WoodFlipoutConstants.k_woodFlipoutD;
      woodFlipoutConfig.Slot0.kS = WoodFlipoutConstants.k_woodFlipoutS;
      woodFlipoutConfig.Slot0.kV = WoodFlipoutConstants.k_woodFlipoutV;
      woodFlipoutConfig.Slot0.kA = WoodFlipoutConstants.k_woodFlipoutA;
      woodFlipoutConfig.Slot0.kG = WoodFlipoutConstants.k_woodFlipoutG;

      woodFlipoutConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      woodFlipoutConfig.CurrentLimits.SupplyCurrentLimit = 60;
      woodFlipoutConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true; 
      woodFlipoutConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Units.Rotations.of(22).in(Units.Rotations);
      woodFlipoutConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
      woodFlipoutConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Units.Rotations.of(-2).in(Units.Rotations); // Starting position
      // woodFlipoutConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

      woodFlipoutConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;


      // Intake Configs
      woodIntakeConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0.05;
      woodIntakeConfig.MotorOutput.PeakForwardDutyCycle = 0.9;
      woodIntakeConfig.MotorOutput.PeakReverseDutyCycle = -0.9;
      woodIntakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      woodIntakeConfig.CurrentLimits.SupplyCurrentLimit = 60;
    }
  }
}