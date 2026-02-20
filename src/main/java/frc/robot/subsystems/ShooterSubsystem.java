package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
  VictorSPX frontIntakeMotor = new VictorSPX(31);
  SparkMax bumperIntakeMotor = new SparkMax(33, MotorType.kBrushless);

  SparkMax hopperIntake = new SparkMax(32, MotorType.kBrushless);

  TalonFX backSpinMotor = new TalonFX(21);
  TalonFX shooterLow = new TalonFX(22);
  TalonFX shooterHigh = new TalonFX(23);

  PIDController shooterSpeedPID = new PIDController(1, 0, 0);

  public Command spoolShootCommand(double speed) {
    return Commands.run(() -> {
      shooterHigh.set(1 * speed);
      backSpinMotor.set(-speed);
    }).finallyDo(() -> {
      shooterHigh.set(0);
      backSpinMotor.set(0);
    });
  }

  public Command shootCommand = Commands.run(() -> {
    shooterLow.set(-0.5);
  }).finallyDo(() -> {
    shooterLow.set(0);
  });

  public Command hopperLoadCommand = Commands.run(() -> {
    hopperIntake.set(-0.25);
  }).finallyDo(() -> {
    hopperIntake.set(0);
  });

  public Command floorPickupCommand = Commands.run(() -> {
    frontIntakeMotor.set(VictorSPXControlMode.PercentOutput, 1);
    bumperIntakeMotor.set(1);
  }).finallyDo(() -> {
    frontIntakeMotor.set(VictorSPXControlMode.PercentOutput, 0);
    bumperIntakeMotor.set(0);
  });
}
