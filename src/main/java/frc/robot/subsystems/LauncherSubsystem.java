// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkMaxRelativeEncoder.Type;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkRelativeEncoder;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
public class LauncherSubsystem extends SubsystemBase {
    
    
    public CANSparkMax followerLauncherR;
    public CANSparkMax feederLauncher;
    public CANSparkMax leaderLauncherL;
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
        followerLauncherR = new CANSparkMax(22, MotorType.kBrushless);
        leaderLauncherL = new CANSparkMax(21, MotorType.kBrushless);

        leaderPIDController = leaderLauncherL.getPIDController();
        followerPIDController = followerLauncherR.getPIDController();
        feederPIDController = feederLauncher.getPIDController();

        feederLauncher.restoreFactoryDefaults();
        followerLauncherR.restoreFactoryDefaults();  //Remove this when we remove the burnFlash() call below
        leaderLauncherL.restoreFactoryDefaults();

        leaderLauncherL.setInverted(false);
        followerLauncherR.follow(leaderLauncherL, true);
        feederLauncher.setInverted(false);

        leaderLauncherL.enableVoltageCompensation(12.0);
        leaderLauncherL.setSmartCurrentLimit(80);
        leaderLauncherL.setIdleMode(IdleMode.kBrake);
        leaderPIDController.setP(kLeaderP,0);
        leaderPIDController.setI(kLeaderI,0);
        leaderPIDController.setD(kLeaderD,0);
        leaderPIDController.setFF(kLeaderFF,0);
        leaderPIDController.setIZone(kLeaderIZone,0);
        leaderPIDController.setOutputRange(kLeaderOutputMin, kLeaderOutputMax,0);
        leaderPIDController.setSmartMotionMaxVelocity(kLeaderMaxRPM,0);
        leaderPIDController.setSmartMotionMinOutputVelocity(kLeaderMinRPM,0);
        leaderPIDController.setSmartMotionMaxAccel(kLeaderMaxAccel,0);
        followerLauncherR.enableVoltageCompensation(12.0);
        followerLauncherR.setSmartCurrentLimit(80);
        followerLauncherR.setIdleMode(IdleMode.kBrake);
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
        leaderLauncherL.burnFlash();
        followerLauncherR.burnFlash();
        feederLauncher.burnFlash();
        leaderEncoder = leaderLauncherL.getEncoder();
        followerEncoder = followerLauncherR.getEncoder();
        feederEncoder = feederLauncher.getEncoder();

        // Add motors to the simulation
        if (Robot.isSimulation()) {
            REVPhysicsSim.getInstance().addSparkMax(leaderLauncherL, 2.6f, 5676.001953f);
            REVPhysicsSim.getInstance().addSparkMax(followerLauncherR, 2.6f, 5676.001953f);
            REVPhysicsSim.getInstance().addSparkMax(feederLauncher, 2.6f, 5676.001953f);
        }
        
    }
    
    // // An accessor method to set the speed (technically the output percentage) of the launch wheel
    public void setLaunchWheel(double speed) {
        // leaderLauncherL.set(speed);
        leaderPIDController.setReference(speed, CANSparkFlex.ControlType.kVelocity);
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

    }
    
    public Command IntakeCmd() {
    return this.startEnd(
        () -> {
            // leaderLauncherL.set(-0.25);
            // feederLauncher.set(-0.25);
            leaderPIDController.setReference(-1000, CANSparkFlex.ControlType.kVelocity);
            feederPIDController.setReference(-1000, CANSparkFlex.ControlType.kVelocity);
        },
        () -> {
            // leaderLauncherL.set(0);
            // feederLauncher.set(0);
            leaderPIDController.setReference(0, CANSparkFlex.ControlType.kVelocity);
            feederPIDController.setReference(0, CANSparkFlex.ControlType.kVelocity);
        });
    }

    public Command AmpCmd() {
    return this.startEnd(
        () -> {
            // leaderLauncherL.set(0.10);
            // feederLauncher.set(0.10);
            leaderPIDController.setReference(500, CANSparkFlex.ControlType.kVelocity);
            feederPIDController.setReference(500, CANSparkFlex.ControlType.kVelocity);
        },
        () -> {
            // leaderLauncherL.set(0);
            // feederLauncher.set(0);
            leaderPIDController.setReference(0, CANSparkFlex.ControlType.kVelocity);
            leaderPIDController.setReference(0, CANSparkFlex.ControlType.kVelocity);
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
            SmartDashboard.putNumber("Lead Launch Wheel Speed (RPM)", leaderEncoder.getVelocity());
            SmartDashboard.putNumber("Follower Launch Wheel Speed (RPM)", followerEncoder.getVelocity());
            SmartDashboard.putNumber("Feed Wheel Speed (RPM)", feederEncoder.getVelocity());
        }
    }
}