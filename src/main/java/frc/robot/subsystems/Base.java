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

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;

public class Base extends SubsystemBase {
  public static final int KDriveRightTopTalon = 6;
  public static final int KDriveLeftTopTalon = 11; 
  public static final int KDriveRightFrontTalon = 3;
  public static final int KDriveLeftFrontTalon = 4;
  public static final int KDriveRightRearTalon = 12; 
  public static final int KDriveLeftRearTalon = 8; 

  private TalonSRX driveRightTop;
  private TalonSRX driveLeftTop;  
  private TalonSRX driveRightFront; 
  private TalonSRX driveLeftFront;
  private TalonSRX driveRightRear; 
  private TalonSRX driveLeftRear;

  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
  private final ColorMatch m_colorMatcher = new ColorMatch();

  private final Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
  private final Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
  private final Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
  private final Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);

  public static double speedLeft = 0;
  public static double speedRight = 0; 

  public Base() {
    driveRightTop = new TalonSRX(KDriveRightTopTalon);
    driveLeftTop = new TalonSRX(KDriveLeftTopTalon);
    driveRightFront = new TalonSRX(KDriveRightFrontTalon); 
    driveLeftFront = new TalonSRX(KDriveLeftFrontTalon);
    driveRightRear = new TalonSRX(KDriveRightRearTalon);
    driveLeftRear = new TalonSRX(KDriveLeftRearTalon);

    driveLeftFront.setNeutralMode(NeutralMode.Brake);
    driveLeftTop.setNeutralMode(NeutralMode.Brake);
    driveLeftRear.setNeutralMode(NeutralMode.Brake);
    driveRightFront.setNeutralMode(NeutralMode.Brake);
    driveRightTop.setNeutralMode(NeutralMode.Brake);
    driveRightRear.setNeutralMode(NeutralMode.Brake);

    driveLeftFront.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 30, 40, 1));
    driveLeftTop.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 30, 40, 1));
    driveLeftRear.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 30, 40, 1));
    driveRightFront.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 30, 40, 1));
    driveRightTop.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 30, 40, 1));
    driveRightRear.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 30, 40, 1));

    driveRightFront.setInverted(true);
    driveRightTop.setInverted(true);
    driveRightRear.setInverted(true);

    driveLeftFront.setInverted(false);
    driveLeftTop.setInverted(false);
    driveLeftRear.setInverted(false);

    driveRightTop.set(ControlMode.Follower, driveRightFront.getDeviceID());
    driveRightRear.set(ControlMode.Follower, driveRightFront.getDeviceID());
    driveLeftTop.set(ControlMode.Follower, driveLeftFront.getDeviceID());
    driveLeftRear.set(ControlMode.Follower, driveLeftFront.getDeviceID());

    m_colorMatcher.addColorMatch(kBlueTarget);
    m_colorMatcher.addColorMatch(kGreenTarget);
    m_colorMatcher.addColorMatch(kRedTarget);
    m_colorMatcher.addColorMatch(kYellowTarget);  
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Base left speed", speedLeft);
    SmartDashboard.putNumber("Base right speed", speedRight);

    Color detectedColor = m_colorSensor.getColor();
    double IR = m_colorSensor.getIR();

    String colorString;
    ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);

    if (match.color == kBlueTarget) {
      colorString = "Blue";
    }
    else if (match.color == kRedTarget) {
      colorString = "Red";
    }
    else if (match.color == kGreenTarget) {
      colorString = "Green";
    }
    else if (match.color == kYellowTarget) {
      colorString = "Yellow";
    }
    else {
      colorString = "Unknown";
    }

    //SmartDashboard.putNumber("Red", detectedColor.red);
    //SmartDashboard.putNumber("Green", detectedColor.green);
    //SmartDashboard.putNumber("Blue", detectedColor.blue);
    //SmartDashboard.putNumber("Confidence", match.confidence);
    //SmartDashboard.putString("Detected Color", colorString);
    //SmartDashboard.putNumber("IR", IR);
    // This method will be called once per scheduler 
  }

  public void tankDrive(double leftSpeed, double rightSpeed) {
    speedLeft = leftSpeed;
    speedRight = rightSpeed;
    driveRightFront.set(ControlMode.PercentOutput, rightSpeed);
    driveLeftFront.set(ControlMode.PercentOutput, leftSpeed);
  }
}
