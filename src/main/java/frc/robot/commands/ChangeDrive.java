package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ChangeDrive extends Command {
  private static boolean originalDrive = false;

  public ChangeDrive() {
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    toggleInvert();
  }

  public static void toggleInvert() {
    originalDrive = !originalDrive;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }

  public static boolean isOriginalDrive() {
    return originalDrive;
  }
}
