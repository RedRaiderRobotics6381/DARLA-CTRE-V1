// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands;

import com.revrobotics.CANSparkBase.ControlType;

// import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Constants;
import frc.robot.subsystems.LauncherSubsystem;

public class SpeakerCmd extends Command{

    private final LauncherSubsystem Launcher;

    public SpeakerCmd(LauncherSubsystem Launcher) {
        this.Launcher = Launcher;
        addRequirements(Launcher);
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
    Launcher.leaderPIDController.setReference(5600, ControlType.kSmartVelocity);
    // System.out.println(Launcher.leaderLauncherL.getEncoder().getVelocity());
    if(Math.abs(Launcher.leaderLauncherL.getEncoder().getVelocity()) >= 5500) {
        // Launcher.feederLauncher.set(1.0);
        Launcher.feederPIDController.setReference(5000, ControlType.kSmartVelocity);
    }


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Launcher.leaderLauncherL.set(0.0);
    // Launcher.feederLauncher.set(0.0);
    Launcher.leaderPIDController.setReference(0, ControlType.kSmartVelocity);
    Launcher.feederPIDController.setReference(0, ControlType.kSmartVelocity);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}