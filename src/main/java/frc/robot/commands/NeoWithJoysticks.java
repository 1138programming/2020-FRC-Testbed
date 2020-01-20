/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.subsystems.Neo;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * An example command that uses an example subsystem.
 */
public class NeoWithJoysticks extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  /**
   * Creates a new NeoWithJoysticks.
   *
   * @param subsystem The subsystem used by this command.
   */
  public NeoWithJoysticks() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.neo);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putNumber("Flywheel top", 0.0);
    SmartDashboard.putNumber("Flywheel bottom", 0.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = Robot.oi.getLeftAxis(); 

    //RobotContainer.neo.move(speed);
    //RobotContainer.neo.move(0.5);
    //RobotContainer.neo.move(SmartDashboard.getNumber("Flywheel top", 0.0), SmartDashboard.getNumber("Flywheel bottom", 0.0));
    RobotContainer.neo.setTopSetpoint(SmartDashboard.getNumber("Flywheel top", 0.0));
    RobotContainer.neo.setBottomSetpoint(SmartDashboard.getNumber("Flywheel bottom", 0.0));
    RobotContainer.neo.calculate();
    SmartDashboard.putNumber("Flywheel", speed);
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
