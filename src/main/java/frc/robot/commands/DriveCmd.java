// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.SteerRequestType;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.FiducialVisionSubsystem;
public class DriveCmd extends Command {
    private final DriveTrainSubsystem m_driveTrain;
    private final DoubleSupplier vX, vY, oX, oY;
    private final IntSupplier POV;
    private final BooleanSupplier lookTarget, hdgMode;

    boolean hdgModePressed = false; // Flag to track button state
    boolean angHdgMode = false; // Flag to track angle mode
    boolean angVelMode = true; // Flag to track velocity mode (default)
    boolean hdgPOV = false; // Flag to track POV mode
    boolean resetHeading = false; // Flag to track heading reset
    boolean holdLastRotation = false; // Flag to track holding last rotation
    Rotation2d holdLastRotationVal = new Rotation2d(); // Variable to hold last rotation

    private Rotation2d rotation = new Rotation2d();
    private Rotation2d rotationPOV = new Rotation2d();
    private Rotation2d rotationStick = new Rotation2d();
    private Rotation2d rotationTarget = new Rotation2d();


    private final SwerveRequest.FieldCentricFacingAngle swerveRequestFacing = new SwerveRequest.FieldCentricFacingAngle()
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
      .withSteerRequestType(SteerRequestType.MotionMagic)
      .withVelocityX(0.0)
      .withVelocityY(0.0);

    private final SwerveRequest.FieldCentric fieldCentric = new SwerveRequest.FieldCentric()
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
      .withSteerRequestType(SteerRequestType.MotionMagic)
      .withVelocityX(0.0)
      .withVelocityY(0.0);

   /** Creates a new DriveCMD.
   * A command that controls the drive train of a swerve drivetrain.
   * This command takes input from various sources (suppliers) to control the velocity and orientation of the drivetrain.
   * It also provides options to control the direction the drivetrain should look at and whether it should be in heading mode.
   *
   * @param m_driveTrain The swerve drivetrain subsystem.
   * @param vX A supplier of the X-axis velocity.
   * @param vY A supplier of the Y-axis velocity.
   * @param oX A supplier of the X-axis rotation.
   * @param oY A supplier of the Y-axis rotation.
   * @param POV The POV value of the controller.
   * @param lookTarget A supplier indicating whether to look at a specific target.
   * @param hdgMode A supplier indicating whether to use heading mode.
   */
  public DriveCmd(DriveTrainSubsystem m_driveTrain, FiducialVisionSubsystem m_fiducialVision, DoubleSupplier vX, DoubleSupplier vY,
                  DoubleSupplier oX, DoubleSupplier oY, IntSupplier POV, BooleanSupplier lookTarget, BooleanSupplier hdgMode) {
    this.m_driveTrain = m_driveTrain;
    // this.m_fiducialVision = m_fiducialVision;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_driveTrain);

    this.vX = vX; this.vY = vY; this.oX = oX; this.oY = oY;
    this.POV = POV;
    this.lookTarget = lookTarget;
    this.hdgMode = hdgMode;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    resetHeading = true;
    angVelMode = true;
    hdgPOV = false;
    swerveRequestFacing.HeadingController = new PhoenixPIDController(5, 0, 0);
    swerveRequestFacing.HeadingController.setTolerance(0.1);
    swerveRequestFacing.HeadingController.enableContinuousInput(-3, 3);
  }

  // Called every time the scheduler runs while the command is scheduled.
  /**
   * Executes the drive command.
   * This method calculates the smoothed X and Y axis velocities based on the input velocities.
   * It also handles the heading mode and rotation logic based on the input values.
   * Finally, it sets the control mode and parameters for the drive train based on the heading mode and target direction.
   */
  @Override
  public void execute() {
    double x = vX.getAsDouble(); // Get the X-axis velocity
    double y = vY.getAsDouble(); // Get the Y-axis velocity
    double root2 = Math.sqrt(2); // Square root of 2
    double magnitude = Math.sqrt(x * x + y * y); // Magnitude of the velocity vector
    double vXsmoothed = Math.signum(x) * Math.pow(Math.min(Math.abs(x * root2), magnitude), Constants.DriveConstants.userInputExponetial); // Smooth the X-axis velocity
    double vYsmoothed = Math.signum(y) * Math.pow(Math.min(Math.abs(y * root2), magnitude), Constants.DriveConstants.userInputExponetial); // Smooth the Y-axis velocity

    hdgPOV = false; // Reset POV mode flag
    
    if (hdgMode.getAsBoolean() && !hdgModePressed) { // Button pressed
      hdgModePressed = true; // Button pressed, set flag to true
      holdLastRotation = true; // Set flag to hold current heading
      holdLastRotationVal = m_driveTrain.getState().Pose.getRotation(); // Store current heading
      if (angHdgMode) { // If in angle mode
        angVelMode = true; // Switch to velocity mode
        angHdgMode = false; // Switch off angle mode
      } else { // If in velocity mode
        angHdgMode = true; // Switch to angle mode
        angVelMode = false; // Switch off velocity mode
      }
    } else if (!hdgMode.getAsBoolean()) { // Button released
      hdgModePressed = false; // Button released, reset flag
    }

    if (angHdgMode) { // If in angle mode
      if (!holdLastRotation && POV.getAsInt() != -1) { // If POV is pressed
          rotationPOV = Rotation2d.fromDegrees(-((double) POV.getAsInt())); // Get POV value
          rotation = rotationPOV; // Set rotation to POV
          rotationStick = null; // Ignore rotationStick
      } else if (!holdLastRotation && Math.hypot(oX.getAsDouble(),oY.getAsDouble()) > .5) { // If right stick is pressed
          double degrees = Math.toDegrees(Math.atan2(oX.getAsDouble(), oY.getAsDouble())); // Get angle of right stick
          rotationStick = Rotation2d.fromDegrees(degrees + 180); // Set rotationStick
          rotation = rotationStick; // Set rotation to right stick
          rotationPOV = null; // Ignore rotationPOV
      } else if (holdLastRotation) { // If holding last rotation
          rotation = holdLastRotationVal; // Set rotation to last rotation
          holdLastRotation = false; // Reset flag
      }
    }

    if (lookTarget.getAsBoolean()) { // If looking at target
      rotationTarget = Robot.m_fiducialVision.getSpeakerYaw(); // Get target rotation
      rotation = rotationTarget; // Set rotation to target
    }
    
    if (angHdgMode || lookTarget.getAsBoolean()) // If in angle mode or looking at the target
    {
      m_driveTrain.setControl(swerveRequestFacing
                              .withDeadband(Constants.DriveConstants.MaxSpeed * 0.1)
                              .withVelocityX(-vYsmoothed * Constants.DriveConstants.MaxSpeed)
                              .withVelocityY(-vXsmoothed * Constants.DriveConstants.MaxSpeed)
                              .withTargetDirection(rotation));
    }

    if (angVelMode && !lookTarget.getAsBoolean()) // If in velocity mode and not looking at the target
    {
      m_driveTrain.setControl(fieldCentric
                              .withDeadband(Constants.DriveConstants.MaxSpeed * 0.1)
                              .withVelocityX(-vYsmoothed * Constants.DriveConstants.MaxSpeed)
                              .withVelocityY(-vXsmoothed * Constants.DriveConstants.MaxSpeed)
                              .withRotationalRate(-oX.getAsDouble() * Constants.DriveConstants.MaxAngularRate));
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}