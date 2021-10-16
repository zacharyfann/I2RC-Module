// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.kauailabs.navx.frc.AHRS;
public class DriveTrain extends SubsystemBase {
private AHRS navx = new AHRS(SPI.Port.kMXP);



  


  

  private final WPI_TalonSRX _leftDriveTalon;
  private final WPI_TalonSRX _righttDriveTalon;

  private DifferentialDrive _diffDrive;

  /** Creates a new DriveTrain. */
  public DriveTrain() {
    _leftDriveTalon = new WPI_TalonSRX(Constants.DriveTrainPorts.LeftDriveTalonPort);
    _righttDriveTalon = new WPI_TalonSRX(Constants.DriveTrainPorts.RightDriveTalonPort);
    _leftDriveTalon.setInverted(false);
    _righttDriveTalon.setInverted(false);
   _leftDriveTalon.configFactoryDefault();
    _leftDriveTalon.setInverted(false);
    _leftDriveTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
    _righttDriveTalon.configFactoryDefault();
    _righttDriveTalon.setInverted(false);
    _righttDriveTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);

    _diffDrive = new DifferentialDrive(_leftDriveTalon, _righttDriveTalon);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void tankDrive(double leftSpeed, double rightSpeed) {
    _diffDrive.tankDrive(leftSpeed, rightSpeed);


  }
  public void resetEncoders() {
    _leftDriveTalon.setSelectedSensorPosition(0, 0, 10);
    _righttDriveTalon.setSelectedSensorPosition(0, 0, 10);
  }
public double getPosition () {
 return   (_leftDriveTalon.getSelectedSensorPosition(0) + (_righttDriveTalon.getSelectedSensorPosition(0)))/2 ;
 
} 
public double getVelocity () {
 return (_leftDriveTalon.getSensorCollection().getPulseWidthVelocity()) + (_leftDriveTalon.getSensorCollection().getPulseWidthVelocity()/2);
}
public void arcadeDrive(double speed, double rotation){
    _diffDrive.arcadeDrive(speed, rotation);
  }
}
}