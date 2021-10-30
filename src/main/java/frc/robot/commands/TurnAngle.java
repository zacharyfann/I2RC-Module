// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class TurnAngle extends CommandBase {
  private final DriveTrain driveTrain;
  private double angle;
  private int constant = 1;
  

  /** Creates a new TimeDrive. */

  public TurnAngle(DriveTrain dt, double NewAngle) {
      driveTrain = dt;
      angle = NewAngle;
      if (angle >= 0) {
        constant = 1;
      } else {
        constant = -1;
      }
      addRequirements(driveTrain);
    }
    // Use addRequirements() here to declare subsystem dependencies.
    

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveTrain.resetGyro();
    
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      driveTrain.tankDrive(0.5 * constant, -0.5 * constant);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.tankDrive(0,0);
    driveTrain.resetEncoders();
  
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Math.abs(driveTrain.getnavAngle()) >= Math.abs(angle)) {
     return true;
    }
    return false;
    
  }
}
