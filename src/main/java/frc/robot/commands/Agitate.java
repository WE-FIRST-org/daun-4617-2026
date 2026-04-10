// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Commands;
// import frc.robot.subsystems.AgitatorSubsystem;
// import static frc.robot.Constants.AgitatorConstants.*;

// /* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
// public class Agitate extends Command {
//   /** Creates a new Intake. */

//   AgitatorSubsystem agitatorSubsystem;

//   public Agitate(AgitatorSubsystem agitatorSystem) {
//     addRequirements(agitatorSystem);
//     this.agitatorSubsystem = agitatorSystem;
//   }

//   // public Command oscilllateCommand() {
//   //   return Commands.sequence(
//   //     agitatorSubsystem.runOnce(() -> agitatorSubsystem.setAgitatorMotor(AGITATOR_FORWARD_MOTOR_VOLTAGE)),
//   //     Commands.waitSeconds(1.5),
//   //     agitatorSubsystem.runOnce(() -> agitatorSubsystem.setAgitatorMotor(AGITATOR_BACKWARD_MOTOR_VOLTAGE)),
//   //     Commands.waitSeconds(1.5)
//   //   ).repeatedly(); 
//   // }

//   // Called when the command is initially scheduled. Set the rollers to the
//   // appropriate values for intaking
//   @Override
//   public void initialize() {
//     // agitatorSubsystem.setAgitatorMotor(AGITATOR_FORWARD_MOTOR_VOLTAGE);
//   }

//   // Called every time the scheduler runs while the command is scheduled. This
//   // command doesn't require updating any values while running
//   @Override
//   public void execute() {

//   }

//   // Called once the command ends or is interrupted. Stop the rollers
//   @Override
//   public void end(boolean interrupted) {
//     agitatorSubsystem.setAgitatorMotor(0);

//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }
