package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.subsystems.IMUSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ResetSensors extends Command {
  private CANDriveSubsystem driveSystem;
  private IMUSubsystem imu;

  public ResetSensors(CANDriveSubsystem driveSystem, IMUSubsystem imu) {
    addRequirements(driveSystem,imu);
    this.driveSystem = driveSystem;
    this.imu = imu;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  @Override 
  public void execute() {
    driveSystem.resetEnconders();
    // imu.resetYaw();
  }  
}
