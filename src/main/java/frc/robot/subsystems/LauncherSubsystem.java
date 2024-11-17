package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class LauncherSubsystem extends SubsystemBase {
    
    
    public CANSparkMax rightLauncher;
    public CANSparkMax bottomLauncher;
    public CANSparkMax leftLauncher;
    public SparkPIDController topLauncherPID;
    public SparkPIDController bottomLauncherPID;

    public LauncherSubsystem() {
        bottomLauncher = new CANSparkMax(Constants.LauncherConstants.kLauncherBottom, MotorType.kBrushless);
        rightLauncher = new CANSparkMax(Constants.LauncherConstants.kLauncherTopRight, MotorType.kBrushless);
        leftLauncher = new CANSparkMax(Constants.LauncherConstants.kLauncherTopLeft, MotorType.kBrushless);

        bottomLauncher.restoreFactoryDefaults();
        rightLauncher.restoreFactoryDefaults();  //Remove this when we remove the burnFlash() call below
        leftLauncher.restoreFactoryDefaults();

        leftLauncher.setInverted(false);
        rightLauncher.follow(leftLauncher, true);
        bottomLauncher.setInverted(false);

        topLauncherPID = leftLauncher.getPIDController();
        bottomLauncherPID = bottomLauncher.getPIDController();

        // initialze PID controller and encoder objects
        // intakePIDController = intakeMotor.getPIDController();
        leftLauncher.enableVoltageCompensation(12.0);
        leftLauncher.setSmartCurrentLimit(80);
        leftLauncher.setIdleMode(IdleMode.kBrake);
        rightLauncher.enableVoltageCompensation(12.0);
        rightLauncher.setSmartCurrentLimit(80);
        rightLauncher.setIdleMode(IdleMode.kBrake);
        bottomLauncher.enableVoltageCompensation(12.0);
        bottomLauncher.setSmartCurrentLimit(80);
        bottomLauncher.setIdleMode(IdleMode.kBrake);
        leftLauncher.burnFlash();
        rightLauncher.burnFlash();
        bottomLauncher.burnFlash();
        
    }
}