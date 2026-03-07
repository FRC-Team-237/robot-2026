// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.vision.Vision;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  private Vision visionLeft;
  private Vision visionRight;

  public Robot() {
    m_robotContainer = new RobotContainer();

    visionLeft = new Vision(
        this.m_robotContainer.drivetrain::addVisionMeasurement,
        "Left",
        new Transform3d(
            Inches.of(10),
            Inches.of(9),
            Inches.of(10.5),
            new Rotation3d(
                Degrees.of(0),
                Degrees.of(-30),
                Degrees.of(45))));

    visionRight = new Vision(
        this.m_robotContainer.drivetrain::addVisionMeasurement,
        "Right",
        new Transform3d(
            Inches.of(10),
            Inches.of(-9),
            Inches.of(10.5),
            new Rotation3d(
                Degrees.of(0),
                Degrees.of(-30),
                Degrees.of(-45))));
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    this.visionLeft.periodic();
    this.visionRight.periodic();
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void disabledExit() {
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().cancel(m_autonomousCommand);
    }
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void teleopExit() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void testExit() {
  }

  @Override
  public void simulationPeriodic() {
  }
}
