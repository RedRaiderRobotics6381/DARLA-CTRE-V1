package frc.robot.Command;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.LauncherSubsystem;

public class IntakeCmd extends Command{

    private final LauncherSubsystem Launcher;

    public IntakeCmd(LauncherSubsystem Launcher) {
        this.Launcher = Launcher;
        addRequirements(Launcher);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Launcher.leftLauncher.set(.5);
    Launcher.bottomLauncher.set(.5);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Launcher.topLauncherPID.setReference(Constants.LauncherConstants.zeroSpeed,  CANSparkMax.ControlType.kSmartMotion);
    Launcher.bottomLauncherPID.setReference(Constants.LauncherConstants.zeroSpeed,  CANSparkMax.ControlType.kSmartMotion);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}