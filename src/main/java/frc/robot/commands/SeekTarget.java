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

public class SeekTarget extends CommandBase {
  /**
   * Creates a new SeekTarget.
   */
  public SeekTarget() {
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
      double adjustment  = 0.0;
      if (!RobotContainer.vision.targetVisible()) {
        adjustment = 0.3; 
      }
      else {
          // run target code
      } 
      double leftSpeed =+ adjustment;
      double rightSpeed =- adjustment;

      //RobotContainer.base.tankDrive(leftSpeed, rightSpeed);
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
