// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkFlex;

// import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
// import frc.robot.Constants;
import frc.robot.subsystems.LauncherSubsystem;

public class SpeakerCmd extends Command{

    private final LauncherSubsystem m_Launcher;

    public SpeakerCmd(LauncherSubsystem m_Launcher) {
        this.m_Launcher = m_Launcher;
        addRequirements(m_Launcher);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // Launcher.topLauncherPID.setReference(Constants.LauncherConstants.topSpeakerSpeed, CANSparkMax.ControlType.kSmartMotion);
    // if(Launcher.leftLauncher.getEncoder().getVelocity() >= 900) {
    //   Launcher.bottomLauncherPID.setReference(Constants.LauncherConstants.bottomSpeakerSpeed, CANSparkMax.ControlType.kSmartMotion);
    // }

    // Launcher.leaderLauncherL.set(1.0);
    m_Launcher.leaderPIDController.setReference(5600, ControlType.kSmartVelocity);
    if (Robot.isSimulation()) {
      m_Launcher.followerPIDController.setReference(5600, CANSparkFlex.ControlType.kVelocity);
    }
    // System.out.println(Launcher.leaderLauncherL.getEncoder().getVelocity());
    if(Math.abs(m_Launcher.leaderLauncher.getEncoder().getVelocity()) >= 5500) {
        // Launcher.feederLauncher.set(1.0);
        m_Launcher.feederPIDController.setReference(5000, ControlType.kSmartVelocity);
    }


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Launcher.leaderLauncherL.set(0.0);
    // Launcher.feederLauncher.set(0.0);
    m_Launcher.leaderPIDController.setReference(0, ControlType.kSmartVelocity);
    m_Launcher.feederPIDController.setReference(0, ControlType.kSmartVelocity);
    if (Robot.isSimulation()) {
      m_Launcher.followerPIDController.setReference(0, CANSparkFlex.ControlType.kVelocity);
  }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}