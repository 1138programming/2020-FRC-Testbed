/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.subsystems.Base;
import frc.robot.subsystems.Vision;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * An example command that uses an example subsystem.
 */
public class PositionBase extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  /**
   * Creates a new PositionBase.
   *
   * @param subsystem The subsystem used by this command.
   */
  public PositionBase() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.base);
    addRequirements(RobotContainer.vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double output = RobotContainer.vision.getXOutput(); 
    if(-0.1 < output && 0.1 > output) {
      RobotContainer.base.tankDrive(0, 0);
    } 
    else if (output > 0.1) {
      RobotContainer.base.tankDrive(-0.5, 0.5);
    } 
    else if (output < -0.1) {
      RobotContainer.base.tankDrive(0.5, -0.5);
    } 

    SmartDashboard.putNumber("XOutputInCommand", output);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }

  public String getName() {
    return "Position base";
  }
}
