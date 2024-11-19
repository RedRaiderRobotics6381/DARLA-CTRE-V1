// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.SpeakerCmd;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LauncherSubsystem;

public class RobotContainer {

  LauncherSubsystem launchersSubsystem = new LauncherSubsystem();
  private double Max_Speed_Multiplier = 0.75; // Default speed multiplier
  private Rotation2d RobotFacingAngle = Rotation2d.fromDegrees(180); // Default robot facing angle

  private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps * Max_Speed_Multiplier; // kSpeedAt12VoltsMps desired top speed
  private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  public final CommandXboxController driverXbox = new CommandXboxController(0); // My joystick 
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

  private SwerveRequest.FieldCentricFacingAngle driveFieldCentricFacingAngle = new SwerveRequest.FieldCentricFacingAngle()
  .withDeadband(MaxSpeed * .1)
  .withRotationalDeadband(MaxAngularRate * .1)
  .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
  .withSteerRequestType(SteerRequestType.MotionMagic)
  .withVelocityX(0.0)
  .withVelocityY(0.0);
  

  private SwerveRequest.FieldCentric driveFC = new SwerveRequest.FieldCentric()
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage);


  private SwerveRequest.RobotCentric driveRo = new SwerveRequest.RobotCentric()
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(.625).withRotationalDeadband(.625) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  // private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  // private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();


  private final Telemetry logger = new Telemetry(MaxSpeed);


  // private Command lockToAngleCommand(double redAngle, double blueAngle) {
  //   return drivetrain.applyRequest(() -> {
  //     double u = driverXbox.getLeftX();
  //     double v = driverXbox.getLeftY();
  //     double root2 = Math.sqrt(2);
  //     double magnitude = Math.sqrt(u * u + v * v);
  //     double x2 = Math.signum(u) * Math.min(Math.abs(u * root2), magnitude);
  //     double y2 = Math.signum(v) * Math.min(Math.abs(v * root2), magnitude);
  //     return facingRequest
  //         .withVelocityX(
  //             modifyAxis(y2)
  //                 * TunerConstants.kSpeedAt12VoltsMps)
  //         .withHeading(edu.wpi.first.math.util.Units.degreesToRadians((DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) ? blueAngle : redAngle))
  //         .withVelocityY(modifyAxis(x2) * TunerConstants.kSpeedAt12VoltsMps);
  //   }).until(() -> Math.abs(driverXbox.getRawAxis(4)) > 0.15);
  // }

  private void configureBindings() {

    //Button 1 is "A" on xbox controller
    //Button 2 is "B" on xbox controller
    //Button 3 is "X" on xbox controller  
    //Button 4 is "Y" on xbox controller
    //Button 5 is "Left Bumper" on xbox controller
    //Button 6 is "Right Bumper" on xbox controller
    //Button 7 is "Back" on xbox controller
    //Button 8 is "Start" on xbox controller
    //Button 9 is "Left Joystick" on xbox controller
    //Button 10 is "Right Joystick" on xbox controller
    //Axis 0 is left joystick x side to side
    //Axis 1 is left joystick y forward and back
    //Axis 2 is left trigger 
    //Axis 3 is right trigger
    //Axis 4 is right joystick x side to side
    //Axis 5 is right joystick y forward and back[\]


    // drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
    //     drivetrain.applyRequest(() -> drive.withVelocityX(Math.pow(-driverXbox.getLeftY(), 3) * MaxSpeed) // Drive forward with negative Y (forward)
    //         .withVelocityY(Math.pow(-driverXbox.getLeftX(), 3) * MaxSpeed) // Drive left with negative X (left)
    //         .withRotationalRate(Math.pow(-driverXbox.getRightX(), 3) * MaxAngularRate)) // Drive counterclockwise with negative X (left)
    // );};
    
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
      drivetrain.applyRequest(() -> {
      driveFieldCentricFacingAngle.HeadingController = new PhoenixPIDController(5, 0, 0); 
      driveFieldCentricFacingAngle.HeadingController.setTolerance(0.1);
      driveFieldCentricFacingAngle.HeadingController.enableContinuousInput(-5, 5);
      double u = driverXbox.getLeftX();
      double v = driverXbox.getLeftY();
      double root2 = Math.sqrt(2);
      double magnitude = Math.sqrt(u * u + v * v);
      double x2 = Math.signum(u) * Math.min(Math.abs(u * root2), magnitude);
      double y2 = Math.signum(v) * Math.min(Math.abs(v * root2), magnitude);
      boolean hdgModePressed = false; // Flag to track button state
      boolean angHdgMode = false; // Flag to track angle mode
      boolean angVelMode = true; // Flag to track velocity mode (default)
      // boolean hdgPOV = false; // Flag to track POV mode
      // boolean resetHeading = false; // Flag to track heading reset 
      //RobotFacingAngle = Rotation2d.fromDegrees(0);
      if (driverXbox.getHID().getPOV() == -1 && driverXbox.getRightX() >= Math.abs(.2) || driverXbox.getRightY() <= Math.abs(.2)){
        RobotFacingAngle = Rotation2d.fromDegrees(Math.toDegrees(Math.atan2(driverXbox.getRightX(), driverXbox.getRightY())));
      }
      driverXbox.pov(0).onTrue(new InstantCommand(() -> {
        RobotFacingAngle = Rotation2d.fromDegrees(0);
      }));
      driverXbox.pov(45).onTrue(new InstantCommand(() -> {
        RobotFacingAngle = Rotation2d.fromDegrees(45);
      }));
      driverXbox.pov(90).onTrue(new InstantCommand(() -> {
        RobotFacingAngle = Rotation2d.fromDegrees(90);
      }));
      driverXbox.pov(135).onTrue(new InstantCommand(() -> {
        RobotFacingAngle = Rotation2d.fromDegrees(135);
      }));
      driverXbox.pov(180).onTrue(new InstantCommand(() -> {
        RobotFacingAngle = Rotation2d.fromDegrees(180);
      }));
      driverXbox.pov(225).onTrue(new InstantCommand(() -> {
        RobotFacingAngle = Rotation2d.fromDegrees(225);
      }));
      driverXbox.pov(270).onTrue(new InstantCommand(() -> {
        RobotFacingAngle = Rotation2d.fromDegrees(270);
      }));
      driverXbox.pov(315).onTrue(new InstantCommand(() -> {
        RobotFacingAngle = Rotation2d.fromDegrees(315);
      }));
      
      if (driverXbox.b().getAsBoolean() && !hdgModePressed) {
        hdgModePressed = true; // Button pressed, set flag to true
        if (angHdgMode) {
          angVelMode = true; // Switch to velocity mode
          angHdgMode = false; // Switch off angle mode
        } else {
          angHdgMode = true; // Switch to angle mode
          angVelMode = false; // Switch off velocity mode
          // This is to prevent the robot from spinning when switching modes
          RobotFacingAngle = drivetrain.getRotation3d().toRotation2d(); // Get current heading
        }
        //System.out.println("hdgMode: " + hdgMode.getAsBoolean() + " angHdgMode: " + angHdgMode + " angVelMode: " + angVelMode); // Debugging
      } else if (!driverXbox.b().getAsBoolean()) {
        hdgModePressed = false; // Button released, reset flag
      }
      
      if (angHdgMode == true) // If in angle mode
      {
        return driveFieldCentricFacingAngle 
          .withVelocityX(-y2 * MaxSpeed)
          .withVelocityY(-x2 * MaxSpeed)
          .withTargetDirection(RobotFacingAngle);      }
      // if (angVelMode == true) // If in velocity mode
      // {
      else{
        return drive 
          .withVelocityX(-y2 * MaxSpeed)
          .withVelocityY(-x2 * MaxSpeed)
          .withRotationalRate(Math.copySign(Math.pow(-driverXbox.getRightX(),2),-driverXbox.getRightX()) * MaxAngularRate);
      }


      // return driveFieldCentricFacingAngle 
      //       .withVelocityX(-y2 * MaxSpeed)
      //       .withVelocityY(-x2 * MaxSpeed)
      //       .withTargetDirection(RobotFacingAngle);
      
    }));

  //   drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
  //   drivetrain.applyRequest(() -> {
  //   double u = driverXbox.getLeftX();
  //   double v = driverXbox.getLeftY();
  //   double root2 = Math.sqrt(2);
  //   double magnitude = Math.sqrt(u * u + v * v);
  //   double x2 = Math.signum(u) * Math.min(Math.abs(u * root2), magnitude);
  //   double y2 = Math.signum(v) * Math.min(Math.abs(v * root2), magnitude);
  //   return drive 
  //         .withVelocityX(-y2 * MaxSpeed)
  //         .withVelocityY(-x2 * MaxSpeed)
  //         .withRotationalRate(Math.copySign(Math.pow(-driverXbox.getRightX(),2),-driverXbox.getRightX()) * MaxAngularRate);
  // }));

    




    

    // joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
    // joystick.b().whileTrue(drivetrain
    //     .applyRequest(() -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

    // reset the field-centric heading on left bumper press
    driverXbox.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    // if (Utils.isSimulation()) {
    //   drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    // }
    drivetrain.registerTelemetry(logger::telemeterize);

    driverXbox.rightBumper().whileTrue(launchersSubsystem.IntakeCmd());
    driverXbox.y().whileTrue(launchersSubsystem.AmpCmd());
    driverXbox.x().whileTrue(new SpeakerCmd(launchersSubsystem));



  }

  public RobotContainer() {
    configureBindings();
  }


  

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
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
      Max_Speed_Multiplier = 1;
    }

    if (driverXbox.getHID().getRightBumper() == true && driverXbox.getHID().getLeftBumper() == false ||
        driverXbox.getHID().getRightBumper() == false && driverXbox.getHID().getLeftBumper() == true){
      //System.out.println("MedSpd");
      Max_Speed_Multiplier = .875;
    }

    if (driverXbox.getHID().getRightBumper() == false && driverXbox.getHID().getLeftBumper() == false){
      //System.out.println("LowSpd");
      Max_Speed_Multiplier = .75;
    }


    // driverXbox.pov(0).onTrue(new InstantCommand(() -> {
    //   RobotFacingAngle = new Rotation2d(0);
    // }));
    // driverXbox.pov(45).onTrue(new InstantCommand(() -> {
    //   RobotFacingAngle = new Rotation2d(45);
    // }));
    // driverXbox.pov(90).onTrue(new InstantCommand(() -> {
    //   RobotFacingAngle = new Rotation2d(90);
    // }));
    // driverXbox.pov(135).onTrue(new InstantCommand(() -> {
    //   RobotFacingAngle = new Rotation2d(135);
    // }));
    // driverXbox.pov(180).onTrue(new InstantCommand(() -> {
    //   RobotFacingAngle = new Rotation2d(180);
    // }));
    // driverXbox.pov(225).onTrue(new InstantCommand(() -> {
    //   RobotFacingAngle = new Rotation2d(225);
    // }));
    // driverXbox.pov(270).onTrue(new InstantCommand(() -> {
    //   RobotFacingAngle = new Rotation2d(270);
    // }));
    // driverXbox.pov(315).onTrue(new InstantCommand(() -> {
    //   RobotFacingAngle = new Rotation2d(315);
    // }));
    // SmartDashboard.putNumber("RobotFacingAngle", RobotFacingAngle.getDegrees());
    
  }

  // private static double deadband(double value, double deadband) {
  //   if (Math.abs(value) > deadband) {
  //     if (value > 0.0) {
  //       return (value - deadband) / (1.0 - deadband);
  //     } else {
  //       return (value + deadband) / (1.0 - deadband);
  //     }
  //   } else {
  //     return 0.0;
  //   }
  // }
}