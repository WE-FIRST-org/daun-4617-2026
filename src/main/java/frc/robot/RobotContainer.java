// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
// import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import static frc.robot.Constants.OperatorConstants.*;

import frc.robot.commands.AimAndRangeCommand;
import frc.robot.commands.Drive;
import frc.robot.commands.Eject;
import frc.robot.commands.ExampleAuto;
import frc.robot.commands.Intake;
import frc.robot.commands.InvertDrive;
import frc.robot.commands.LaunchSequence;
import frc.robot.commands.ResetSensors;
// import frc.robot.commands.RotateToAngle;
import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.subsystems.CANFuelSubsystem;
import frc.robot.subsystems.IMUSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final IMUSubsystem imu = new IMUSubsystem();
  private final CANDriveSubsystem driveSubsystem = new CANDriveSubsystem(imu);
  private final CANFuelSubsystem fuelSubsystem = new CANFuelSubsystem();
  public final VisionSubsystem m_visionSubsystem = new VisionSubsystem();

  // The driver's controller
  private final CommandXboxController driverController = new CommandXboxController(
      DRIVER_CONTROLLER_PORT);

  // The operator's controller
  private final CommandXboxController operatorController = new CommandXboxController(
      OPERATOR_CONTROLLER_PORT);

  // The autonomous chooser
  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    configureBindings();

    // Set the options to show up in the Dashboard for selecting auto modes. If you
    // add additional auto modes you can add additional lines here with
    // autoChooser.addOption
    autoChooser.setDefaultOption("ScanForTag", new ExampleAuto(driveSubsystem, fuelSubsystem));
    autoChooser.addOption("LaunchSequence", new LaunchSequence(fuelSubsystem));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the {@link Trigger#Trigger(java.util.function.BooleanSupplier)}
   * constructor with an arbitrary predicate, or via the named factories in
   * {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses
   * for {@link CommandXboxController Xbox}/
   * {@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    driverController.leftTrigger(0.1).whileTrue(new AimAndRangeCommand(driveSubsystem, m_visionSubsystem));
    // Run invert once when the bumper is pressed (whileTrue schedules repeatedly)
    driverController.leftBumper().onTrue(new InvertDrive());

  // rotate to relative headings: +180, -90, +90 degrees from current
  // driverController.y().onTrue(new RotateToAngle(driveSubsystem, imu, 180.0));
  // driverController.x().onTrue(new RotateToAngle(driveSubsystem, imu, -90.0));
  // driverController.b().onTrue(new RotateToAngle(driveSubsystem, imu, 90.0));

    // While the left bumper on operator controller is held, intake Fuel
    operatorController.leftBumper().whileTrue(new Intake(fuelSubsystem));
    // While the right bumper on the operator controller is held, spin up for 1
    // second, then launch fuel. When the button is released, stop.
    operatorController.rightBumper().whileTrue(new LaunchSequence(fuelSubsystem));
    // While the A button is held on the operator controller, eject fuel back out
    // the intake
    operatorController.a().whileTrue(new Eject(fuelSubsystem));

    // Set the default command for the drive subsystem to the command provided by
    // factory with the values provided by the joystick axes on the driver
    // controller. The Y axis of the controller is inverted so that pushing the
    // stick away from you (a negative value) drives the robot forwards (a positive
    // value)
    driveSubsystem.setDefaultCommand(new Drive(driveSubsystem, driverController));
    fuelSubsystem.setDefaultCommand(fuelSubsystem.run(() -> fuelSubsystem.stop()));

    // finding feedforward constant
    // driverController.y().whileTrue(driveSubsystem.sysIdQuasistaticLinear(Direction.kForward));
    // driverController.a().whileTrue(driveSubsystem.sysIdQuasistaticLinear(Direction.kReverse));
    // driverController.b().whileTrue(driveSubsystem.sysIdDynamicLinear(Direction.kForward));
    // driverController.x().whileTrue(driveSubsystem.sysIdDynamicLinear(Direction.kReverse));
    // driverController.povUp().whileTrue(driveSubsystem.sysIdQuasistaticAngular(Direction.kForward));
    // driverController.povDown().whileTrue(driveSubsystem.sysIdQuasistaticAngular(Direction.kReverse));
    // driverController.povRight().whileTrue(driveSubsystem.sysIdDynamicAngular(Direction.kForward));
    // driverController.povLeft().whileTrue(driveSubsystem.sysIdDynamicAngular(Direction.kReverse));

    driverController.start().onTrue(new ResetSensors(driveSubsystem, imu));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   * This functions is run at each cycle during the autonomous period. 
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    //Get status of the robot, then based on the status of the robot, return the auto command
    double launcher_speed = fuelSubsystem.getLauncherSpeed();
    int shooterApril_tag = m_visionSubsystem.getBestShooterTarget().getFiducialId();
    
    
    return null;
  }
}