/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
//import com.revrobotics.*;
//import com.revrobotics.CANSparkMax;
//import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.controller.TakeBackHalf;

public class Neo extends SubsystemBase {

    //public static final int KTopSparkMax = 11;
    //public static final int KBottomSparkMax = 10;
    public static final int KTopTalon = 12;
    public static final int KBottomTalon = 8;

    public static final double KTopGain = 0.000006;
    public static final double KBottomGain = 0.000006;

    //private CANSparkMax TopMotor; 
	  //private CANSparkMax BottomMotor; 
    private TalonSRX TopMotor;
    private TalonSRX BottomMotor;

    private TakeBackHalf topController;
    private TakeBackHalf bottomController;

    private double topSpeed;
    private double bottomSpeed;

    public Neo () {
		  // TopMotor = new CANSparkMax(TopSparkMax, CANSparkMaxLowLevel.MotorType.kBrushless);
      // BottomMotor = new CANSparkMax(BottomSparkMax, CANSparkMaxLowLevel.MotorType.kBrushless);
      TopMotor = new TalonSRX(KTopTalon);
      BottomMotor = new TalonSRX(KBottomTalon);

      TopMotor.setInverted(true);
      BottomMotor.setInverted(false);

      TopMotor.setSensorPhase(true);

      topController = new TakeBackHalf(KTopGain);
      bottomController = new TakeBackHalf(KBottomGain);

      //SmartDashboard.putNumber("Take Back Half gain", 0.000006); // ~0.000006 is best
      topController.setInputRange(-80000, 80000);
      topController.setOutputRange(-1, 1);

      bottomController.setInputRange(-80000, 80000);
      bottomController.setOutputRange(-1, 1);
    }

  @Override
  public void periodic() {
    // This method will be called once per scheduler 

    //SmartDashboard.putNumber("Flywheel top setpoint", tbhController.getSetpoint());
    //SmartDashboard.putNumber("Flywheel top speed", getTopSpeed());
    //SmartDashboard.putNumber("Flywheel top error", tbhController.getError());
    //SmartDashboard.putNumber("Fylwheel top PWM", topSpeed);
    //SmartDashboard.putNumber("Flywheel top h0", tbhController.getH0());
  }

  /*public void neoMotorMove(double speed) {
    NeoMotor1.set(speed);
    NeoMotor2.set(speed);
	}*/
  /*public void neoMotorStop() {
    NeoMotor1.set(0);
    NeoMotor2.set(0);
  }*/
  
  public void move(double topSpeed, double bottomSpeed) {
    TopMotor.set(ControlMode.PercentOutput, topSpeed);
    BottomMotor.set(ControlMode.PercentOutput, bottomSpeed);
  }

  public double getTopSpeed() {
    return TopMotor.getSelectedSensorVelocity();
  }

  public double getBottomSpeed() {
    return BottomMotor.getSelectedSensorVelocity();
  }

  public double getTopSetpoint() {
    //return tbhController.getSetpoint();
    return topController.getSetpoint();
  }

  public double getBottomSetpoint() {
    //return tbhController.getSetpoint();
    return topController.getSetpoint();
  }

  public void setSetpoint(double topSetpoint, double bottomSetpoint) {
    topController.setSetpoint(topSetpoint);
    bottomController.setSetpoint(bottomSetpoint);
  }
  
  public void calculate() {
    topSpeed = topController.calculate(getTopSpeed());
    bottomSpeed = bottomController.calculate(getBottomSpeed());
    move(topSpeed, bottomSpeed);
  }
}
