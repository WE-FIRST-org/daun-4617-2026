// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.AgitatorConstants.*;

public class AgitatorSubsystem extends SubsystemBase {
  private final SparkMax agitatorLeader;
  // private final SparkMax agitatorFollower;

  /** Creates a new CANBallSubsystem. */
  public AgitatorSubsystem() {
    // create brushless motors for each of the motors on the launcher mechanism
    agitatorLeader = new SparkMax(AGITATOR_LEADER_ID, MotorType.kBrushed);
    // agitatorFollower = new SparkMax(AGITATOR_FOLLOWER_ID, MotorType.kBrushed);

    // create the configuration for the feeder roller, set a current limit and apply
    // the config to the controller
    SparkMaxConfig agitatorConfig = new SparkMaxConfig();
    agitatorConfig.smartCurrentLimit(AGITATOR_MOTOR_CURRENT_LIMIT);
    agitatorLeader.configure(agitatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // agitatorConfig.follow(agitatorLeader);
    // agitatorFollower.configure(agitatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  // A method to set the voltage of the intake roller
  public void setAgitatorMotor(double voltage) {
    agitatorLeader.setVoltage(voltage);
  }


  // A method to stop the rollers
  public void stop() {
    agitatorLeader.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public Command oscilllateCommand() {
    return Commands.sequence(
      this.runOnce(() -> this.setAgitatorMotor(AGITATOR_FORWARD_MOTOR_VOLTAGE)),
      Commands.waitSeconds(1.5),
      this.runOnce(() -> this.setAgitatorMotor(AGITATOR_BACKWARD_MOTOR_VOLTAGE)),
      Commands.waitSeconds(1.5)
    ).repeatedly(); 
  }
}
