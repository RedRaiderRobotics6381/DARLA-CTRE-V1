// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.REVPhysicsSim;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
public class LauncherSubsystem extends SubsystemBase {
    
    
    public CANSparkMax followerLauncherR;
    public CANSparkMax feederLauncher;
    public CANSparkMax leaderLauncherL;
    
    public LauncherSubsystem() {
        feederLauncher = new CANSparkMax(20, MotorType.kBrushless);
        followerLauncherR = new CANSparkMax(22, MotorType.kBrushless);
        leaderLauncherL = new CANSparkMax(21, MotorType.kBrushless);

        feederLauncher.restoreFactoryDefaults();
        followerLauncherR.restoreFactoryDefaults();  //Remove this when we remove the burnFlash() call below
        leaderLauncherL.restoreFactoryDefaults();

        leaderLauncherL.setInverted(false);
        followerLauncherR.follow(leaderLauncherL, true);
        feederLauncher.setInverted(false);

        leaderLauncherL.enableVoltageCompensation(12.0);
        leaderLauncherL.setSmartCurrentLimit(80);
        leaderLauncherL.setIdleMode(IdleMode.kBrake);
        followerLauncherR.enableVoltageCompensation(12.0);
        followerLauncherR.setSmartCurrentLimit(80);
        followerLauncherR.setIdleMode(IdleMode.kBrake);
        feederLauncher.enableVoltageCompensation(12.0);
        feederLauncher.setSmartCurrentLimit(80);
        feederLauncher.setIdleMode(IdleMode.kBrake);
        leaderLauncherL.burnFlash();
        followerLauncherR.burnFlash();
        feederLauncher.burnFlash();


        // Add motors to the simulation
        if (Robot.isSimulation()) {
            REVPhysicsSim.getInstance().addSparkMax(leaderLauncherL, 2.6f, 5676.001953f);
            REVPhysicsSim.getInstance().addSparkMax(followerLauncherR, 2.6f, 5676.001953f);
            REVPhysicsSim.getInstance().addSparkMax(feederLauncher, 2.6f, 5676.001953f);
        }
        
    }
    
    // An accessor method to set the speed (technically the output percentage) of the launch wheel
    public void setLaunchWheel(double speed) {
        leaderLauncherL.set(speed);
    }

    // An accessor method to set the speed (technically the output percentage) of the feed wheel
    public void setFeedWheel(double speed) {
        feederLauncher.set(speed);
    }

    

    // A helper method to stop both wheels. You could skip having a method like this and call the
    // individual accessors with speed = 0 instead
    public void stop() {
        leaderLauncherL.set(0);
        feederLauncher.set(0);
    }
    
    public Command IntakeCmd() {
    return this.startEnd(
        () -> {
            leaderLauncherL.set(-0.25);
            feederLauncher.set(-0.25);
        },
        () -> {
            leaderLauncherL.set(0);
            feederLauncher.set(0);
        });
    }

    public Command AmpCmd() {
    return this.startEnd(
        () -> {
            leaderLauncherL.set(0.10);
            feederLauncher.set(0.10);
        },
        () -> {
            leaderLauncherL.set(0);
            feederLauncher.set(0);
        });
    }
    
    // Method to update simulation data
    public void simulationPeriodic() {
        if (Robot.isSimulation()) {
            REVPhysicsSim.getInstance().run();

            // Display simulation data on the SmartDashboard
            SmartDashboard.putNumber("Lead Launch Wheel Speed", leaderLauncherL.get());
            SmartDashboard.putNumber("Follower Launch Wheel Speed", followerLauncherR.get());
            SmartDashboard.putNumber("Feed Wheel Speed", feederLauncher.get());
            // Display simulation data on the SmartDashboard
        SmartDashboard.putNumber("Lead Launch Wheel Speed (RPM)", leaderLauncherL.getEncoder().getVelocity());
        SmartDashboard.putNumber("Follower Launch Wheel Speed (RPM)", followerLauncherR.getEncoder().getVelocity());
        SmartDashboard.putNumber("Feed Wheel Speed (RPM)", feederLauncher.getEncoder().getVelocity());
        }
    }
}