package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
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

        leftLauncher.setInverted(true);
        rightLauncher.follow(leftLauncher, true);

        topLauncherPID = leftLauncher.getPIDController();
        bottomLauncherPID = bottomLauncher.getPIDController();

        // initialze PID controller and encoder objects
        // intakePIDController = intakeMotor.getPIDController();
        leftLauncher.enableVoltageCompensation(12.0);
        leftLauncher.setSmartCurrentLimit(60);
        rightLauncher.enableVoltageCompensation(12.0);
        rightLauncher.setSmartCurrentLimit(60);
        bottomLauncher.enableVoltageCompensation(12.0);
        bottomLauncher.setSmartCurrentLimit(80);
        bottomLauncher.burnFlash();
        
    }
}