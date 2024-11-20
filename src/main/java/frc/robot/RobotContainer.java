// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
// import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.DriveCmd;
import frc.robot.commands.MusicCmd;
import frc.robot.commands.SpeakerCmd;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.FiducialVisionSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
public class RobotContainer {
  /* Local Constants */
  private final double MaxSpeed = Constants.DriveConstants.MaxSpeed;

  /* Controllers */
  public final CommandXboxController driverXbox = new CommandXboxController(0); // My joystick 

  /*  Subsytems */
  public final DriveTrainSubsystem drivetrain = TunerConstants.DriveTrain; // My drivetrain  public final CommandXboxController driverXbox = new CommandXboxController(0); // My joystick 
  public final LauncherSubsystem launchersSubsystem = new LauncherSubsystem();
  private final Telemetry logger = new Telemetry(MaxSpeed);
  private  FiducialVisionSubsystem fiducialVision;

  /*  Commands */
  public MusicCmd lalala = new MusicCmd(drivetrain);
  private final SendableChooser<Command> autoChooser;
  private final DriveCmd c_Drive = new DriveCmd(drivetrain,
                                                fiducialVision,
                                                () -> driverXbox.getLeftX(),
                                                () -> driverXbox.getLeftY(),
                                                () -> driverXbox.getRightX(),
                                                () -> driverXbox.getRightY(),
                                                () -> driverXbox.getHID().getPOV(),
                                                () -> driverXbox.b().getAsBoolean(),
                                                () -> driverXbox.rightStick().getAsBoolean());

  // private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  // private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  private void configureBindings() {

    //Button 1 is "A" on xbox controller - Used by IntakeCMD
    //Button 2 is "B" on xbox controller - Used by DriveCMD
    //Button 3 is "X" on xbox controller - Used by SpeakerCMD
    //Button 4 is "Y" on xbox controller - Used by AmpCMD
    //Button 5 is "Left Bumper" on xbox controller - Used by SpencerButtons
    //Button 6 is "Right Bumper" on xbox controller - Used by SpencerButtons
    //Button 7 is "Back" on xbox controller - Used by Music
    //Button 8 is "Start" on xbox controller - Used by ResetFieldRelative
    //Button 9 is "Left Joystick" on xbox controller
    //Button 10 is "Right Joystick" on xbox controller - Used by DriveCMD
    //Axis 0 is left joystick x side to side - Used by DriveCMD
    //Axis 1 is left joystick y forward and back - Used by DriveCMD
    //Axis 2 is left trigger 
    //Axis 3 is right trigger
    //Axis 4 is right joystick x side to side - Used by DriveCMD
    //Axis 5 is right joystick y forward and back - Used by DriveCMD
    
    drivetrain.setDefaultCommand(c_Drive); // Drivetrain will execute this command periodically

    driverXbox.back().onTrue(lalala); // Pressing "B" on the xbox controller will play music

    // joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
    // joystick.b().whileTrue(drivetrain
    //     .applyRequest(() -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

    
    // Pressing "Start" on the xbox controller will reset the field-centric heading
    driverXbox.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));
    

    // if (Utils.isSimulation()) {
    //   drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    // }

    drivetrain.registerTelemetry(logger::telemeterize); // Register the telemetry function to be called periodically

    /* Bindings for drivetrain characterization */
    /* These bindings require multiple buttons pushed to swap between quastatic and dynamic */
    /* Back/Start select dynamic/quasistatic, Y/X select forward/reverse direction */
    // driverXbox.back().and(driverXbox.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    // driverXbox.back().and(driverXbox.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    // driverXbox.start().and(driverXbox.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    // driverXbox.start().and(driverXbox.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

    // Holding the right bumper on the xbox controller will intake note
    driverXbox.a().whileTrue(launchersSubsystem.IntakeCmd());
    // Holding "Y" on the xbox controller will shoot note at low power
    driverXbox.y().whileTrue(launchersSubsystem.AmpCmd());
    // Holding "X" on the xbox controller will shoot note at full power
    driverXbox.x().whileTrue(new SpeakerCmd(launchersSubsystem));
    driverXbox.leftStick().whileTrue(Commands.deferredProxy(() -> drivetrain.driveToPose(
                              FiducialVisionSubsystem.getAprilTagPose(Constants.AprilTagConstants.ampID,
                                                            new Transform2d(0.5, 0,
                                                            Rotation2d.fromDegrees(180))))));
  }

  public RobotContainer() {
    configureBindings();

    NamedCommands.registerCommand("Intake", launchersSubsystem.IntakeCmd());
    NamedCommands.registerCommand("Amp", launchersSubsystem.AmpCmd());
    NamedCommands.registerCommand("Speaker", new SpeakerCmd(launchersSubsystem));
    autoChooser = AutoBuilder.buildAutoChooser();
    
    SmartDashboard.putData("Auto Chooser", autoChooser);
    
  }

  public Command getAutonomousCommand() {
    /* First put the drivetrain into auto run mode, then run the auto */
    return autoChooser.getSelected();
  }

  /**
   * Sets the speed multiplier for the drivebase based on the state of the right and left bumpers on the driver's Xbox controller.
   * If both bumpers are pressed, the speed multiplier is set to 1 (HighSpd).
   * If either bumper is pressed, the speed multiplier is set to 0.875 (MedSpd).
   * If neither bumper is pressed, the speed multiplier is set to 0.75 (LowSpd).
  */
  public void spencerButtons(){
    if (driverXbox.getHID().getRightBumper() == true && driverXbox.getHID().getLeftBumper() == true){
      //System.out.println("HighSpd");
      Constants.DriveConstants.Max_Speed_Multiplier = 1;
    }

    if (driverXbox.getHID().getRightBumper() == true && driverXbox.getHID().getLeftBumper() == false ||
        driverXbox.getHID().getRightBumper() == false && driverXbox.getHID().getLeftBumper() == true){
      //System.out.println("MedSpd");
      Constants.DriveConstants.Max_Speed_Multiplier = .875;
    }

    if (driverXbox.getHID().getRightBumper() == false && driverXbox.getHID().getLeftBumper() == false){
      //System.out.println("LowSpd");
      Constants.DriveConstants.Max_Speed_Multiplier = .75;
    }
  }

}