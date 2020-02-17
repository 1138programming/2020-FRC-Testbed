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
import frc.robot.controller.TakeBackHalf;

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
    RobotContainer.neo.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //double speed = Robot.oi.getLeftAxis(); 

    //RobotContainer.neo.move(speed);
    //RobotContainer.neo.move(1, 1);

    RobotContainer.neo.setTopConstants(
      SmartDashboard.getNumber("Flywheel Top P", 0.0),
      SmartDashboard.getNumber("Flywheel Top I", 0.0),
      SmartDashboard.getNumber("Flywheel Top D", 0.0),
      SmartDashboard.getNumber("Flywheel Top F", 0.0)
    );

    RobotContainer.neo.setBottomConstants(
      SmartDashboard.getNumber("Flywheel Bottom P", 0.0),
      SmartDashboard.getNumber("Flywheel Bottom I", 0.0),
      SmartDashboard.getNumber("Flywheel Bottom D", 0.0),
      SmartDashboard.getNumber("Flywheel Bottom F", 0.0)
    );

    RobotContainer.neo.setSetpoint(SmartDashboard.getNumber("Flywheel Top Setpoint", 0.0), SmartDashboard.getNumber("Flywheel Bottom Setpoint", 0.0));
    RobotContainer.neo.calculate();
    //SmartDashboard.putNumber("Get Error", TakeBackHalf.getError());
    //SmartDashboard.putNumber("Get m_H0", getm_H0());
    //SmartDashboard.putNumber("Flywheel", speed);
  
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
