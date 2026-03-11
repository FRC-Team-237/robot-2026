// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.HangerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class RobotContainer {
  // private final Telemetry logger = new Telemetry(MaxSpeed);

  private final CommandXboxController joystick = new CommandXboxController(0);
  // controller with overides and reverse buttons for intake
  private final Joystick extra = new Joystick(1);
  private final JoystickButton a = new JoystickButton(extra, 2);
  private final JoystickButton b = new JoystickButton(extra, 3);
  private final JoystickButton c = new JoystickButton(extra, 5);
  private final JoystickButton x = new JoystickButton(extra, 1);

  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  ShooterSubsystem shooter = new ShooterSubsystem(drivetrain);
  HangerSubsystem hanger = new HangerSubsystem();

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    a.whileTrue(shooter.reverseIntakeCommand);
    b.whileTrue(shooter.reverseShootCommand);
    c.whileTrue(shooter.stopIntakeCommand);
    x.whileTrue(drivetrain.aimFieldOrientedCommand(
        () -> -joystick.getLeftY(),
        () -> -joystick.getLeftX()));

    joystick.povUp().whileTrue(hanger.hangerUpCommand);

    joystick.povDown().whileTrue(hanger.hangerDownCommand);

    drivetrain.setDefaultCommand(
        drivetrain.teleopDriveCommand(
            () -> -joystick.getLeftY(),
            () -> -joystick.getLeftX(),
            () -> -joystick.getRightX()));

    final var idle = new SwerveRequest.Idle();
    RobotModeTriggers.disabled()
        .whileTrue(drivetrain.applyRequest(() -> idle).ignoringDisable(true));

    joystick.leftTrigger().whileTrue(shooter.intakeCommand);
    joystick.rightBumper().whileTrue(shooter.shootCommand);
    joystick.rightTrigger().whileTrue(shooter.spoolShootCommand(() -> drivetrain.getState().Pose.getTranslation()));

    // joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
    // joystick.b().whileTrue(drivetrain.applyRequest(() -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

    // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single log.
    // joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    // joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    // joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    // joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

    // Reset the field-centric heading on left bumper press.
    // joystick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

    joystick.a().whileTrue(
        drivetrain.aimZeldaCommand(
            () -> -joystick.getLeftY(),
            () -> -joystick.getLeftX()));
    joystick.b().whileTrue(
        drivetrain.aimFieldOrientedCommand(
            () -> -joystick.getLeftY(),
            () -> -joystick.getLeftX()));

    joystick.y().onTrue(Commands.parallel(
        shooter.spoolShootCommand(() -> drivetrain.getState().Pose.getTranslation()),
        drivetrain.aim().andThen(
            shooter.shootCommand)));

    try {
      var testPath = PathPlannerPath.fromPathFile("New New Path");
      joystick.x().whileTrue(AutoBuilder.followPath(testPath));
    } catch (Exception e) {
      System.out.println("Failed to load path");
    }

    // drivetrain.registerTelemetry(logger::telemeterize);
  }

  public Command getAutonomousCommand() {
    return drivetrain.applyRequest(() -> new SwerveRequest.Idle());
  }
}