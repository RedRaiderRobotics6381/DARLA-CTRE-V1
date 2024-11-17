package frc.robot.Command;

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

    Launcher.leftLauncher.set(1.0);
    System.out.println(Launcher.leftLauncher.getEncoder().getVelocity());
    if(Math.abs(Launcher.leftLauncher.getEncoder().getVelocity()) >= 5500) {
        Launcher.bottomLauncher.set(1.0);
    }


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Launcher.leftLauncher.set(0.0);
    Launcher.bottomLauncher.set(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}