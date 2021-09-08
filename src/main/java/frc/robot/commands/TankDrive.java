// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class TankDrive extends CommandBase {
  /** Creates a new TankDrive. */
  DriveTrain driveTrain;
  Joystick leftJoystick;
  Joystick rightJoystick;


  public TankDrive(DriveTrain dt, Joystick ljs, Joystick rjs) {
    // Use addRequirements() here to declare subsystem dependencies.
    driveTrain = dt;
    leftJoystick = ljs;
    rightJoystick = rjs;

    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double leftPower = -.8 * leftJoystick.getRawAxis(1);
    double rightPower = -.8 * rightJoystick.getRawAxis(1);

    driveTrain.TankDrive(leftPower, rightPower);

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
