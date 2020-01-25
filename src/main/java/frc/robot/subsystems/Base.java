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

import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

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

  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
  private final ColorMatch m_colorMatcher = new ColorMatch();

  private final Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
  private final Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
  private final Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
  private final Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);

  private final AHRS ahrs;
  
  private static double yawAngle;
  private static double velocityX;
  private static double velocityY;
  private static double velocityZ;
  private static double displacementX;
  private static double displacementY;
  private static double displacementZ;

  public Base() {
    driveRightTop = new TalonSRX(KDriveRightTopTalon);
    driveLeftTop = new TalonSRX(KDriveLeftTopTalon);
    driveRightFront = new TalonSRX(KDriveRightFrontTalon); 
    driveLeftFront = new TalonSRX(KDriveLeftFrontTalon);
    driveRightRear = new TalonSRX(KDriveRightRearTalon);
    driveLeftRear = new TalonSRX(KDriveLeftRearTalon);
    ahrs = new AHRS(SPI.Port.kMXP);
  

    driveRightFront.setInverted(true);
    driveRightRear.setInverted(true);
    driveRightTop.setInverted(true);

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

    SmartDashboard.putNumber("Red", detectedColor.red);
    SmartDashboard.putNumber("Green", detectedColor.green);
    SmartDashboard.putNumber("Blue", detectedColor.blue);
    SmartDashboard.putNumber("Confidence", match.confidence);
    SmartDashboard.putString("Detected Color", colorString);
    SmartDashboard.putNumber("IR", IR);
    SmartDashboard.putNumber("FacingAngle", yawAngle);
    SmartDashboard.putNumber("VelocityX", velocityX);
    SmartDashboard.putNumber("VelocityY", velocityY);
    SmartDashboard.putNumber("VelocityZ", velocityZ);
    SmartDashboard.putNumber("DisplacementX", displacementX);
    SmartDashboard.putNumber("DisplacementY", displacementY);
    SmartDashboard.putNumber("DisplacementZ", displacementZ);
    // This method will be called once per scheduler 
  }

  public void tankDrive(double leftSpeed, double rightSpeed) {
    driveRightFront.set(ControlMode.PercentOutput, rightSpeed);
    driveLeftFront.set(ControlMode.PercentOutput, leftSpeed);
  }
  public void yawReset(){
    ahrs.zeroYaw();
  }

  public double getFacingDirection(){
    yawAngle = ahrs.getAngle();
    return yawAngle;
  }

  public double getVelocityX(){
    velocityX = ahrs.getVelocityX();
    return velocityX;
  }

  public double getVelocityY(){
    velocityY = ahrs.getVelocityY();
    return velocityY;
  }

  public double getVelocityZ(){
    velocityZ = ahrs.getVelocityZ();
    return velocityZ;
  }

  public double getDisplacementX(){
    displacementX = ahrs.getDisplacementX();
    return displacementX;
  }

  public double getDisplacementY(){
    displacementY = ahrs.getDisplacementY();
    return displacementY;
  }

  public double getDisplacementZ(){
    displacementZ = ahrs.getDisplacementZ();
    return displacementZ;
  }
}
