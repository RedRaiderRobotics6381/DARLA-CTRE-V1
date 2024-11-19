// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.LEDsSubSystem;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  public static LEDsSubSystem m_LEDsSubSystem = new LEDsSubSystem();

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
        // Silence CAN bus errors during simulation
    DriverStation.silenceJoystickConnectionWarning(true);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {
    m_LEDsSubSystem.scanEffect(60, 255, 255);
  }

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    m_LEDsSubSystem.setSolidLED(120, 255, 255); // Set LEDs to Green with scan effect
  }

  @Override
  public void teleopPeriodic() {
    
    if(DriverStation.getAlliance().get() == Alliance.Blue){
      m_LEDsSubSystem.setSolidLED(90, 255, 255); // Set LEDs to Blue
    } else {
      m_LEDsSubSystem.setSolidLED(0, 255, 255); // Set LEDs to Red
    }
    m_robotContainer.spencerButtons();
    // SmartDashboard.putNumber("X",m_robotContainer.joystick.getLeftX());
    // SmartDashboard.putNumber("Y",m_robotContainer.joystick.getLeftY());
  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {

  }
}