/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import java.lang.Math;

public class Vision extends SubsystemBase {

  public static NetworkTableEntry tx;
  public static NetworkTableEntry ty;
  public static NetworkTableEntry ta;
  public static NetworkTableEntry ts;

  public static double x;
  public static double y; 
  public static double area;
  public static double angle;

  public static double currentX;
  public static double currentY;

  public static final double kP = 0.1; 
  public static final double kI = 0.05;
  public static final double kD = 0.2; 
  
  public static double xError; 
  public static double yError;
  public static double baseError;

  public static double xOutput;
  public static double yOutput;

  public static double h1 = 31;
  public static double h2 = 87;
  public static double a1 = 8.877568714;
  public static double a2;

  public static double atest1 = Math.toRadians(a1);
  public static double atest2;
  public static double tan;
  public static double lastDistance = 0;
  public Vision() {
    
  }

  @Override
  public void periodic() {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    ta = table.getEntry("ta");
    ts = table.getEntry("ts");

    x = tx.getDouble(0.0);
    y = ty.getDouble(0.0);
    area = ta.getDouble(0.0);
    angle = ts.getDouble(0.0);

    a2 = y;

    //SmartDashboard.putNumber("test", 1);

    //SmartDashboard.putNumber("LimelightX", x);
    //SmartDashboard.putNumber("LimelightY", y);
    //SmartDashboard.putNumber("LimelightArea", area);
    //SmartDashboard.putNumber("LimelightAngle", angle);
    //SmartDashboard.putNumber("XOutput", getXOutput());

    //SmartDashboard.putNumber("a1", atest1);
    //SmartDashboard.putNumber("a2", atest2);
    //SmartDashboard.putNumber("tan", tan);

    //SmartDashboard.putNumber("Distance From Tower", getDistance());
  }

  public double getXOutput() {
    currentX = x;
    xError = 0.0 - currentX;
    xOutput = kP * xError + kI + kD;
    return xOutput;
  }

  public double getYOutput() {
    currentY = y;
    yError = 0.0 - currentY;
    yOutput = kP * yError + kI + kD;
    return yOutput;
  }

  public double getDistance() {
    //double hypotenuse = (6.0033*(area * area)) - (65.33*area) + 264.82;
    //double distance = Math.sqrt(((hypotenuse * hypotenuse) - (49 * 49)));
    //return distance/12;
    if (area > 0.1) {
      atest2 = Math.toRadians(a2);
      tan = Math.tan(atest1+atest2);
    
      double distance = 54/tan;
      lastDistance = distance;

      return distance; 
    } else {
      return lastDistance;
    }
  }
}
