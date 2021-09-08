// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {
  /** Creates a new DriveTrain. */
  WPI_TalonSRX leftMotor;
  WPI_TalonSRX rightMotor;

  DifferentialDrive diffDrive;

  public DriveTrain() {
    leftMotor = new WPI_TalonSRX(Constants.DrivePorts.leftDrivePort);
    rightMotor = new WPI_TalonSRX(Constants.DrivePorts.rightDriveport);

    diffDrive = new DifferentialDrive(leftMotor, rightMotor);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void TankDrive(double leftPower, double rightPower) {
    diffDrive.tankDrive(leftPower, rightPower);
  }

}
