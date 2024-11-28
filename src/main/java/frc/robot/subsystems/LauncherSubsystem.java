// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
public class LauncherSubsystem extends SubsystemBase {
    
    
    public CANSparkMax followerLauncher;
    public CANSparkMax feederLauncher;
    public CANSparkMax leaderLauncher;
    private RelativeEncoder leaderEncoder;
    private RelativeEncoder followerEncoder;
    private RelativeEncoder feederEncoder;
    public SparkPIDController leaderPIDController;
    public SparkPIDController followerPIDController;
    public SparkPIDController feederPIDController;
    // private double feederLauncherSpeed = 0.0;
    // private double leaderLauncherSpeed = 0.0;
    // private double followerLauncherSpeed = 0.0;
    private double kLeaderP = 0.0005, kLeaderI = 0.0, kLeaderD = 0.0;
    private double kFollowerP = 0.0005, kFollowerI = 0.0, kFollowerD = 0.0;
    private double kFeederP = 0.0005, kFeederI = 0.0, kFeederD = 0.0;
    private double kLeaderFF = 0.0005, kFollowerFF = 0.0005, kFeederFF = 0.0005;
    private double kLeaderIZone = 0.0, kFollowerIZone = 0.0, kFeederIZone = 0.0;
    private double kLeaderOutputMin = -1.0, kFollowerOutputMin = -1.0, kFeederOutputMin = -1.0;
    private double kLeaderOutputMax = 1.0, kFollowerOutputMax = 1.0, kFeederOutputMax = 1.0;
    private double kLeaderMaxRPM = 5676, kFollowerMaxRPM = 5676, kFeederMaxRPM = 5676;
    private double kLeaderMinRPM = 0.0, kFollowerMinRPM = 0.0, kFeederMinRPM = 0.0;
    private double kLeaderMaxAccel = 10000, kFollowerMaxAccel = 10000, kFeederMaxAccel = 10000;

    public LauncherSubsystem() {
        feederLauncher = new CANSparkMax(20, MotorType.kBrushless);
        followerLauncher = new CANSparkMax(22, MotorType.kBrushless);
        leaderLauncher = new CANSparkMax(21, MotorType.kBrushless);

        leaderPIDController = leaderLauncher.getPIDController();
        followerPIDController = followerLauncher.getPIDController();
        feederPIDController = feederLauncher.getPIDController();

        feederLauncher.restoreFactoryDefaults();
        followerLauncher.restoreFactoryDefaults();  //Remove this when we remove the burnFlash() call below
        leaderLauncher.restoreFactoryDefaults();

        leaderLauncher.setInverted(false);
        followerLauncher.follow(leaderLauncher, true);
        feederLauncher.setInverted(false);

        leaderLauncher.enableVoltageCompensation(12.0);
        leaderLauncher.setSmartCurrentLimit(80);
        leaderLauncher.setIdleMode(IdleMode.kBrake);
        leaderPIDController.setP(kLeaderP,0);
        leaderPIDController.setI(kLeaderI,0);
        leaderPIDController.setD(kLeaderD,0);
        leaderPIDController.setFF(kLeaderFF,0);
        leaderPIDController.setIZone(kLeaderIZone,0);
        leaderPIDController.setOutputRange(kLeaderOutputMin, kLeaderOutputMax,0);
        leaderPIDController.setSmartMotionMaxVelocity(kLeaderMaxRPM,0);
        leaderPIDController.setSmartMotionMinOutputVelocity(kLeaderMinRPM,0);
        leaderPIDController.setSmartMotionMaxAccel(kLeaderMaxAccel,0);
        followerLauncher.enableVoltageCompensation(12.0);
        followerLauncher.setSmartCurrentLimit(80);
        followerLauncher.setIdleMode(IdleMode.kBrake);
        followerPIDController.setP(kFollowerP,0);
        followerPIDController.setI(kFollowerI,0);
        followerPIDController.setD(kFollowerD,0);
        followerPIDController.setFF(kFollowerFF,0);
        followerPIDController.setIZone(kFollowerIZone,0);
        followerPIDController.setOutputRange(kFollowerOutputMin, kFollowerOutputMax,0);
        followerPIDController.setSmartMotionMaxVelocity(kFollowerMaxRPM,0);
        followerPIDController.setSmartMotionMinOutputVelocity(kFollowerMinRPM,0);
        followerPIDController.setSmartMotionMaxAccel(kFollowerMaxAccel,0);
        feederLauncher.enableVoltageCompensation(12.0);
        feederLauncher.setSmartCurrentLimit(80);
        feederLauncher.setIdleMode(IdleMode.kBrake);
        feederPIDController.setP(kFeederP,0);
        feederPIDController.setI(kFeederI,0);
        feederPIDController.setD(kFeederD,0);
        feederPIDController.setFF(kFeederFF,0);
        feederPIDController.setIZone(kFeederIZone,0);
        feederPIDController.setOutputRange(kFeederOutputMin, kFeederOutputMax,0);
        feederPIDController.setSmartMotionMaxVelocity(kFeederMaxRPM,0);
        feederPIDController.setSmartMotionMinOutputVelocity(kFeederMinRPM,0);
        feederPIDController.setSmartMotionMaxAccel(kFeederMaxAccel,0);
        leaderLauncher.burnFlash();
        followerLauncher.burnFlash();
        feederLauncher.burnFlash();
        leaderEncoder = leaderLauncher.getEncoder();
        followerEncoder = followerLauncher.getEncoder();
        feederEncoder = feederLauncher.getEncoder();

        // Add motors to the simulation
        if (Robot.isSimulation()) {
            REVPhysicsSim.getInstance().addSparkMax(leaderLauncher, 2.6f, 5676.001953f);
            REVPhysicsSim.getInstance().addSparkMax(followerLauncher, 2.6f, 5676.001953f);
            REVPhysicsSim.getInstance().addSparkMax(feederLauncher, 2.6f, 5676.001953f);
        }
        
    }
    
    // // An accessor method to set the speed (technically the output percentage) of the launch wheel
    public void setLaunchWheel(double speed) {
        // leaderLauncherL.set(speed);
        leaderPIDController.setReference(speed, CANSparkFlex.ControlType.kVelocity);
        if (Robot.isSimulation()) {
            followerPIDController.setReference(speed, CANSparkFlex.ControlType.kVelocity);
        }
    }

    // An accessor method to set the speed (technically the output percentage) of the feed wheel
    public void setFeedWheel(double speed) {
        // feederLauncher.set(speed);
        feederPIDController.setReference(speed, CANSparkFlex.ControlType.kVelocity);
    }

    

    // A helper method to stop both wheels. You could skip having a method like this and call the
    // individual accessors with speed = 0 instead
    public void stop() {
        // leaderLauncherL.set(0);
        // feederLauncher.set(0);
        leaderPIDController.setReference(0, CANSparkFlex.ControlType.kVelocity);
        feederPIDController.setReference(0, CANSparkFlex.ControlType.kVelocity);
        if (Robot.isSimulation()) {
            followerPIDController.setReference(0, CANSparkFlex.ControlType.kVelocity);
        }

    }
    
    public Command IntakeCmd() {
    return this.startEnd(
        () -> {
            // leaderLauncherL.set(-0.25);
            // feederLauncher.set(-0.25);
            leaderPIDController.setReference(-1000, CANSparkFlex.ControlType.kVelocity);
            feederPIDController.setReference(-1000, CANSparkFlex.ControlType.kVelocity);
            if (Robot.isSimulation()) {
                followerPIDController.setReference(-1000, CANSparkFlex.ControlType.kVelocity);
            }
        },
        () -> {
            // leaderLauncherL.set(0);
            // feederLauncher.set(0);
            leaderPIDController.setReference(0, CANSparkFlex.ControlType.kVelocity);
            feederPIDController.setReference(0, CANSparkFlex.ControlType.kVelocity);
            if (Robot.isSimulation()) {
                followerPIDController.setReference(0, CANSparkFlex.ControlType.kVelocity);
            }
        });
    }

    public Command AmpCmd() {
    return this.startEnd(
        () -> {
            // leaderLauncherL.set(0.10);
            // feederLauncher.set(0.10);
            leaderPIDController.setReference(500, CANSparkFlex.ControlType.kVelocity);
            feederPIDController.setReference(500, CANSparkFlex.ControlType.kVelocity);
            if (Robot.isSimulation()) {
                followerPIDController.setReference(500, CANSparkFlex.ControlType.kVelocity);
            }
        },
        () -> {
            // leaderLauncherL.set(0);
            // feederLauncher.set(0);
            leaderPIDController.setReference(0, CANSparkFlex.ControlType.kVelocity);
            leaderPIDController.setReference(0, CANSparkFlex.ControlType.kVelocity);
            if (Robot.isSimulation()) {
                followerPIDController.setReference(0, CANSparkFlex.ControlType.kVelocity);
            }
        });
    }
    
    @Override
    public void simulationPeriodic() {
        if (Robot.isSimulation()) {
            REVPhysicsSim.getInstance().run();
        }
    }

    @Override //Q: what does override mean?  
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("Lead Launch Wheel Speed (RPM)", leaderEncoder.getVelocity());
        SmartDashboard.putNumber("Follower Launch Wheel Speed (RPM)", followerEncoder.getVelocity());
        SmartDashboard.putNumber("Feed Wheel Speed (RPM)", feederEncoder.getVelocity());
    }
}