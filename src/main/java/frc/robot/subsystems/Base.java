/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.commands.DriveWithJoysticks;
import frc.robot.commands.TestCommand;
import frc.robot.commands.TestCommand2;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Base extends SubsystemBase {

  public static final int KDriveRightTopTalon = 4;
  public static final int KDriveLeftTopTalon = 9; 
  public static final int KDriveRightFrontTalon = 1;
  public static final int KDriveLeftFrontTalon = 5; 
  public static final int KDriveRightRearTalon = 2; 
  public static final int KDriveLeftRearTalon = 8; 

  private TalonSRX driveRightTop;
  private TalonSRX driveLeftTop;  
  private TalonSRX driveRightFront; 
  private TalonSRX driveLeftFront;
  private TalonSRX driveRightRear; 
  private TalonSRX driveLeftRear;

  public Base() {
    driveRightTop = new TalonSRX(KDriveRightTopTalon);
    driveLeftTop = new TalonSRX(KDriveLeftTopTalon);
    driveRightFront = new TalonSRX(KDriveRightFrontTalon); 
    driveLeftFront = new TalonSRX(KDriveLeftFrontTalon);
    driveRightRear = new TalonSRX(KDriveRightRearTalon);
    driveLeftRear = new TalonSRX(KDriveLeftRearTalon);

    driveRightFront.setInverted(true);
    driveRightRear.setInverted(true);
    driveRightTop.setInverted(true);

    driveRightTop.set(ControlMode.Follower, driveRightFront.getDeviceID());
    driveRightRear.set(ControlMode.Follower, driveRightFront.getDeviceID());
    driveLeftTop.set(ControlMode.Follower, driveLeftFront.getDeviceID());
    driveLeftRear.set(ControlMode.Follower, driveLeftFront.getDeviceID());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler 
  }

  public void tankDrive(double leftSpeed, double rightSpeed) {
    driveRightFront.set(ControlMode.PercentOutput, rightSpeed);
    driveLeftFront.set(ControlMode.PercentOutput, leftSpeed);
  }
}
