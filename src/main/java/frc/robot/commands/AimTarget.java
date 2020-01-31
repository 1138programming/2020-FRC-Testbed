/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Base;
import frc.robot.Robot;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AimTarget extends CommandBase {
  /**
   * Creates a new AimTarget.
   */
  public AimTarget() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.vision);
    addRequirements(RobotContainer.base);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      double kp = -0.1; // constant proportional
      double min_command = 0.05; // represents the minimum amount of power needed for the robot to actually move
      double tx = RobotContainer.vision.getXOffset();
      double heading_error = -tx;
      double steering_adjust = 0.0;
      double leftSpeed = 0.0;
      double rightSpeed = 0.0;

      if (tx > 1.0) {
          steering_adjust = kp * heading_error - min_command;
      }
      else if (tx < 1.0) {
          steering_adjust = kp*heading_error + min_command;
      }
      
      leftSpeed += steering_adjust;
      rightSpeed -= steering_adjust;

      //RobotContainer.base.tankDrive(leftSpeed, rightSpeed); 
      SmartDashboard.putNumber("Left LL Aim Correction Speed", leftSpeed); // temp
      SmartDashboard.putNumber("Right LL Aim Correction Speed", rightSpeed); // temp
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
