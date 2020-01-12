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

  public static final int KFalcon1 = 0;
  public static final int KFalcon2 = 1;
  public static final int KFalcon3 = 2;
  public static final int KFalcon4 = 3;
  public static final int KFalcon5 = 4;
  public static final int KFalcon6 = 5;

  private TalonSRX Falcon0;
  private TalonSRX Falcon1;
  private TalonSRX Falcon2;
  private TalonSRX Falcon3;
  private TalonSRX Falcon4;
  private TalonSRX Falcon5;


  public Falcon() {
    Falcon0 = new TalonSRX(KFalcon1);
    Falcon1 = new TalonSRX(KFalcon2);
    Falcon2 = new TalonSRX(KFalcon3);
    Falcon3 = new TalonSRX(KFalcon4);
    Falcon4 = new TalonSRX(KFalcon5);
    Falcon5 = new TalonSRX(KFalcon6);
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler 
  }

  public void move(double speed) {
    Falcon0.set(ControlMode.PercentOutput, speed);
    Falcon1.set(ControlMode.PercentOutput, speed);
    Falcon2.set(ControlMode.PercentOutput, speed);
    Falcon3.set(ControlMode.PercentOutput, speed);
    Falcon4.set(ControlMode.PercentOutput, speed);
    Falcon5.set(ControlMode.PercentOutput, speed);
  }
}
