package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Positions;

public class ShooterSubsystem extends SubsystemBase {
  VictorSPX frontIntakeMotor = new VictorSPX(31);
  SparkMax bumperIntakeMotor = new SparkMax(33, MotorType.kBrushless);

  SparkMax hopperIntake = new SparkMax(32, MotorType.kBrushless);

  TalonFX shooterBack = new TalonFX(21);
  TalonFX shooterFeeder = new TalonFX(22);
  TalonFX shooterFront = new TalonFX(23);

  private static final double FEEDER_DUTY_CYCLE = 1;
  private static final double HOPPER_SPEED = 1.0;

  private CommandSwerveDrivetrain drivetrain;

  private boolean inTheMiddleOfTheField = false;
  private boolean holdingIntakeStopButton = false;
  private boolean driverIntaking = false;
  private double DISTANCE_THRESHOLD = 5;
  private Trigger intakeTrigger = new Trigger(() -> {
    var robotPose = this.drivetrain.getState().Pose;
    var robotXInches = Inches.convertFrom(robotPose.getX(), Meters);
    inTheMiddleOfTheField = robotXInches > 182.11 && robotXInches < 469.11;

    boolean shouldIntake = false;
    if (inTheMiddleOfTheField) {
      shouldIntake = true;
    }

    if (holdingIntakeStopButton) {
      shouldIntake = false;
    }

    if (driverIntaking) {
      shouldIntake = true;
    }

    return shouldIntake;
  });

  public ShooterSubsystem(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;

    var config = new TalonFXConfiguration();
    config.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.5;
    config.CurrentLimits.SupplyCurrentLimit = 100.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    config.CurrentLimits.StatorCurrentLimit = 100.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    config.Slot0.kV = 0.1316;
    config.Slot0.kP = 0.02;
    config.Slot0.kA = 0.01;

    this.shooterFront.getConfigurator().apply(config);
    this.shooterBack.getConfigurator().apply(config);

    var feederConfig = new TalonFXConfiguration();
    feederConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.5;
    feederConfig.CurrentLimits.SupplyCurrentLimit = 120.0;
    feederConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    feederConfig.CurrentLimits.StatorCurrentLimit = 120.0;
    feederConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    feederConfig.CurrentLimits.SupplyCurrentLowerLimit = 80.0;
    feederConfig.CurrentLimits.SupplyCurrentLowerTime = 1.0;

    feederConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    this.shooterFeeder.getConfigurator().apply(feederConfig);

    var intakeConfig = new SparkMaxConfig();
    intakeConfig.smartCurrentLimit(30, 30);

    intakeTrigger.whileTrue(Commands.run(() -> {
      frontIntakeMotor.set(VictorSPXControlMode.PercentOutput, 1);
      bumperIntakeMotor.set(1.0);
      hopperIntake.set(-HOPPER_SPEED);
    }).finallyDo(() -> {
      frontIntakeMotor.set(VictorSPXControlMode.PercentOutput, 0);
      bumperIntakeMotor.set(0);
      hopperIntake.set(0);
    }));

    SmartDashboard.putNumber("FrontSpeed", 0);
    SmartDashboard.putNumber("BackSpeed", 0);
  }

  final VelocityVoltage velocityControl = new VelocityVoltage(0).withEnableFOC(false);

  private double backspinSpeed(double dist) {
    double m;
    double b;
    if (dist <= DISTANCE_THRESHOLD) {
      m = 2.01358;
      b = 3.57923;
    } else {
      m = 1.34024;
      b = 35.66391;
    }
    return m * dist + b;
  }

  private double frontSpinSpeed(double dist) {
    double m;
    double b;
    if (dist <= DISTANCE_THRESHOLD) {
      m = 1.37966;
      b = 59.0729;
    } else {
      m = 1.83487;
      b = 27.69051;
    }
    return m * dist + b;
  }

  // private double backspinSpeed(double dist) {
  // // if (dist > 7.5) {
  // // double m = 1.39241425;
  // // double b = 7.724755125;
  // // return m * dist + b;
  // // }
  // // double m = 2.2278628;
  // // double b = 1.458891;
  // // return m * dist + b;
  // }

  // private double frontSpinSpeed(double dist) {
  // // if (dist > 7.5) {
  // // double m = 1.113522;
  // // double b = 63.2425628;
  // // return m * dist + b;
  // // }
  // // double m = 1.29168552;
  // // double b = 61.9063364;
  // // return m * dist + b;
  // }

  private double voltageAjuster() {
    double voltage = RobotController.getBatteryVoltage();
    double center = 11.5;
    double steepness = 8;
    double maxBoost = 0.075;
    double denominator = 1 + (Math.pow(Math.E, steepness * (voltage - center)));
    return maxBoost / denominator + 1;
  }

  public Command spoolShootCommand(Supplier<Translation2d> robotPosition) {
    return Commands.run(() -> {
      var latestPos = robotPosition.get();
      var targetPos = Positions.myHubPosition(this.drivetrain.getState());
      var distanceToTarget = Feet.convertFrom(latestPos.getDistance(targetPos), Meters);

      double targetSpeedFront = frontSpinSpeed(distanceToTarget);
      double targetSpeedBack = backspinSpeed(distanceToTarget);
      SmartDashboard.putNumber("FrontSpeed", targetSpeedFront);
      SmartDashboard.putNumber("BackSpeed", targetSpeedBack);
      SmartDashboard.putNumber("Distance", distanceToTarget);

      // var targetSpeedFront = SmartDashboard.getNumber("FrontSpeed", 0);
      // var targetSpeedBack = SmartDashboard.getNumber("BackSpeed", 0);
      SmartDashboard.putNumber("Distance", distanceToTarget);

      var voltageAdjustmentRatio = voltageAjuster();

      SmartDashboard.putNumber("VoltageAdjustment", voltageAdjustmentRatio);

      this.shooterFront.setControl(velocityControl.withVelocity(targetSpeedFront * voltageAdjustmentRatio));
      this.shooterBack.setControl(velocityControl.withVelocity(-targetSpeedBack * voltageAdjustmentRatio));
    }).finallyDo(() -> {
      shooterFront.set(0);
      shooterBack.set(0);
    });
  }

  public Command setDistanceSpoolShootCommand() {
    return Commands.run(() -> {
      var targetSpeedFront = frontSpinSpeed(6);
      var targetSpeedBack = backspinSpeed(6);

      var voltageAdjustmentRatio = voltageAjuster();

      SmartDashboard.putNumber("VoltageAdjustment", voltageAdjustmentRatio);

      this.shooterFront.setControl(velocityControl.withVelocity(targetSpeedFront * voltageAdjustmentRatio));
      this.shooterBack.setControl(velocityControl.withVelocity(-targetSpeedBack * voltageAdjustmentRatio));
    }).finallyDo(() -> {
      shooterFront.set(0);
      shooterBack.set(0);
    });
  }

  private double getHopperSpeed() {
    var time = Timer.getFPGATimestamp();
    var TIME_IN = 1.0;
    var TIME_OUT = 0.25;

    var t = time % (TIME_IN + TIME_OUT);
    var sign = t < TIME_IN ? 1.0 : -1.0;

    return HOPPER_SPEED * sign;
  }

  public Command shootCommand() {
    return Commands.run(() -> {
      shooterFeeder.set(-FEEDER_DUTY_CYCLE);
      hopperIntake.set(-getHopperSpeed());
    }).finallyDo(() -> {
      shooterFeeder.set(0);
      hopperIntake.set(0);
    });
  }

  public Command reverseShootCommand() {
    return Commands.run(() -> {
      shooterFeeder.set(FEEDER_DUTY_CYCLE);
    }).finallyDo(() -> {
      shooterFeeder.set(0);
    });
  }

  public Command reverseIntakeCommand() {
    return Commands.run(() -> {
      frontIntakeMotor.set(VictorSPXControlMode.PercentOutput, -1);
      bumperIntakeMotor.set(-1);
      hopperIntake.set(HOPPER_SPEED);
    }).finallyDo(() -> {
      frontIntakeMotor.set(VictorSPXControlMode.PercentOutput, 0);
      bumperIntakeMotor.set(0);
      hopperIntake.set(0);
    });
  }

  public Command driverIntakeCommand() {
    return Commands.run(() -> {
      driverIntaking = true;
    }).finallyDo(() -> {
      driverIntaking = false;
    });
  }

  public Command buttonOverrideIntakeCommand() {
    return Commands.run(() -> {
      holdingIntakeStopButton = true;
    }).finallyDo(() -> {
      holdingIntakeStopButton = false;
    });
  }

  public Command aimAndShoot() {
    return Commands.parallel(
        this.spoolShootCommand(() -> this.drivetrain.getState().Pose.getTranslation()),
        Commands.parallel(
            this.drivetrain.aim(),
            Commands.waitSeconds(0.5)).andThen(
                Commands.parallel(
                    drivetrain.turtleModeCommand(),
                    this.shootCommand())));
  }
}
