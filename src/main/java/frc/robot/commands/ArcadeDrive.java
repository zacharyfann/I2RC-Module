// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;

public class ArcadeDrive extends CommandBase {
  /** Creates a new ArcadeDrive. */
  public DriveTrain driveTrain;
  public Joystick joyStick; 
  public ArcadeDrive(DriveTrain dt,Joystick js) {
    // Use addRequirements() here to declare subsystem dependencies.
    driveTrain = dt;
    joyStick = js;

    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = -1 * joyStick.getRawAxis(Constants.JoystickAxis.YAxis);
    double rotation = joyStick.getRawAxis(Constants.JoystickAxis.XAxis);
    driveTrain.arcadeDrive(speed, rotation);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
