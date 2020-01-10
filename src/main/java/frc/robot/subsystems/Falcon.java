/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import frc.robot.commands.FalconStop;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Falcon extends SubsystemBase {

  public static final int KFalcon = 0;

  private TalonSRX Falcon;

  public Falcon() {
    Falcon = new TalonSRX(KFalcon);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler 
  }

  public void move(double speed) {
    Falcon.set(ControlMode.PercentOutput, speed);
  }
}
