// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import org.photonvision.PhotonCamera;

import com.ctre.phoenix.led.LarsonAnimation.BounceMode;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.AprilTagConstants;
import frc.robot.subsystems.CANdleSubSystem;
import frc.robot.subsystems.FiducialVisionSubsystem;
import frc.robot.subsystems.LEDsSubSystem;
import frc.robot.subsystems.ObjectVisionSubsystem;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  // private RobotContainer m_robotContainer;
  // private ObjectVision m_objectVision;
  // public FiducialVision m_fiducialVision;

  public static LEDsSubSystem m_LEDsSubSystem = new LEDsSubSystem();
  private final CANdleSubSystem m_CANdleSubSystem = new CANdleSubSystem();
  public static PhotonCamera camObj = new PhotonCamera("camObj"); // Create a new PhotonCamera object
  public static PhotonCamera camAprTg = new PhotonCamera("camAprTg"); // Create a new PhotonCamera object
  public static RobotContainer m_robotContainer = new RobotContainer();
  public static ObjectVisionSubsystem m_objectVision = new ObjectVisionSubsystem(m_LEDsSubSystem);
  public static FiducialVisionSubsystem m_fiducialVision = new FiducialVisionSubsystem(m_robotContainer.drivetrain);


  

  @Override
  public void robotInit() {

    DriverStation.silenceJoystickConnectionWarning(true); // Silence the joystick connection warning
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    SmartDashboard.putBoolean("camAprTg Connected",camAprTg.isConnected()); // Check if the camera is connected
    SmartDashboard.putBoolean("camAprTg has Targets",camAprTg.getLatestResult().hasTargets()); // Check if the camera has targets
    m_fiducialVision.updateVisionField(); // Update the vision field
    m_fiducialVision.updatePoseEstimation(m_robotContainer.drivetrain); // Update the pose estimation from the fiducial vision
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {
    m_LEDsSubSystem.scanEffect(282, 255, 211);
    m_CANdleSubSystem.LarsonAnimation(148, 0, 211, 0, 0.5, BounceMode.Front, 5);// Set LEDs to Violet with bounce effect
  }

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
    aprilTagAlliance();
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
    if(DriverStation.getAlliance().get() == Alliance.Blue){
      m_LEDsSubSystem.setSolidLED(90, 255, 255); // Set LEDs to Blue
      m_CANdleSubSystem.setLEDsColor(0, 0, 255, 0, 0.5);

    } else {
      m_LEDsSubSystem.setSolidLED(0, 255, 255); // Set LEDs to Red
      m_CANdleSubSystem.setLEDsColor(255, 0, 0, 0, 0.5);
    }
    aprilTagAlliance();
  }

  @Override
  public void teleopPeriodic() {
    
    if(DriverStation.getAlliance().get() == Alliance.Blue){
      m_LEDsSubSystem.scanEffect(90, 255, 255); // Set LEDs to Blue
      m_CANdleSubSystem.LarsonAnimation(0, 0, 255, 0, 0.5, BounceMode.Front, 5);// Set LEDs to Violet with bounce effect
      // m_LEDsSubSystem.setSolidLED(90, 255, 255); // Set LEDs to Blue
      // m_CANdleSubSystem.setLEDsColor(0, 0, 255, 0, 0.5);

    } else {
      m_LEDsSubSystem.scanEffect(0, 255, 255); // Set LEDs to Red
      m_CANdleSubSystem.LarsonAnimation(255, 0, 0, 0, 0.5, BounceMode.Front, 5);// Set LEDs to Violet with bounce effect
      // m_LEDsSubSystem.setSolidLED(0, 255, 255); // Set LEDs to Red
      // m_CANdleSubSystem.setLEDsColor(255, 0, 0, 0, 0.5);
    }
    m_robotContainer.spencerButtons();
    m_objectVision.watchForNote();
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
  public static void aprilTagAlliance(){
    AprilTagConstants.ampID     = DriverStation.getAlliance().get() == Alliance.Blue ? 6 : 5;
    AprilTagConstants.speakerID = DriverStation.getAlliance().get() == Alliance.Blue ? 7 : 4;
    AprilTagConstants.stageIDA  = DriverStation.getAlliance().get() == Alliance.Blue ? 14 : 13;
    AprilTagConstants.stageIDB  = DriverStation.getAlliance().get() == Alliance.Blue ? 15 : 12;
    AprilTagConstants.stageIDC  = DriverStation.getAlliance().get() == Alliance.Blue ? 16 : 11;
    AprilTagConstants.sourceA  = DriverStation.getAlliance().get() == Alliance.Blue ? 9 : 2;
    AprilTagConstants.sourceB  = DriverStation.getAlliance().get() == Alliance.Blue ? 10 : 1;
  }
}