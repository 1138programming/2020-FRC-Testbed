/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/* We like to party.                                                          */
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
import frc.robot.controller.PIDController;
import frc.robot.controller.LinearProfiler;
import frc.robot.enums.IntegralType;

public class Neo extends SubsystemBase {

    //public static final int KTopSparkMax = 11;
    //public static final int KBottomSparkMax = 10;
    public static final int KTopTalon = 12;
    public static final int KBottomTalon = 8;

    public static final double KTopGain = 0.000006;
    public static final double KBottomGain = 0.000006;

    //private CANSparkMax topMotor; 
	  //private CANSparkMax bottomMotor; 
    private TalonSRX topMotor;
    private TalonSRX bottomMotor;

    //private TakeBackHalf topProfiler;
    //private TakeBackHalf bottomProfiler;
    private LinearProfiler topProfiler;
    private LinearProfiler bottomProfiler;

    private double topSpeed;
    private double bottomSpeed;

    public Neo () {
		  // topMotor = new CANSparkMax(TopSparkMax, CANSparkMaxLowLevel.MotorType.kBrushless);
      // bottomMotor = new CANSparkMax(BottomSparkMax, CANSparkMaxLowLevel.MotorType.kBrushless);
      topMotor = new TalonSRX(KTopTalon);
      bottomMotor = new TalonSRX(KBottomTalon);

      topMotor.setInverted(true);
      bottomMotor.setInverted(false);

      topMotor.setSensorPhase(true);
      bottomMotor.setSensorPhase(true);

      //topProfiler = new TakeBackHalf(KTopGain);
      //bottomProfiler = new TakeBackHalf(KBottomGain);
      topProfiler = new LinearProfiler(20000, 10000, 0, 0, 0, 0.000027, 0, 0.2);
      bottomProfiler = new LinearProfiler(20000, 10000, 0, 0, 0, 0.000027, 0, 0.2);

      //SmartDashboard.putNumber("Take Back Half gain", 0.000006); // ~0.000006 is best
      topProfiler.setOutputRange(-1, 1);
      topProfiler.setTolerance(50, 0);

      bottomProfiler.setOutputRange(-1, 1);
      bottomProfiler.setTolerance(50, 0);

      // Initialize SmartDashboard fields to get numbers from
      SmartDashboard.putNumber("Flywheel Top Target", 0.0);
      SmartDashboard.putNumber("Flywheel Bottom Target", 0.0);
      SmartDashboard.putNumber("Flywheel Top P", topProfiler.getP());
      SmartDashboard.putNumber("Flywheel Top I", topProfiler.getI());
      SmartDashboard.putNumber("Flywheel Top D", topProfiler.getD());
      SmartDashboard.putNumber("Flywheel Top F V", topProfiler.getVelocityFeedforward());
      SmartDashboard.putNumber("Flywheel Top F A", topProfiler.getAccelFeedforward());
      SmartDashboard.putNumber("Flywheel Bottom P", bottomProfiler.getP());
      SmartDashboard.putNumber("Flywheel Bottom I", bottomProfiler.getI());
      SmartDashboard.putNumber("Flywheel Bottom D", bottomProfiler.getD());
      SmartDashboard.putNumber("Flywheel Bottom F V", bottomProfiler.getVelocityFeedforward());
      SmartDashboard.putNumber("Flywheel Bottom F A", bottomProfiler.getAccelFeedforward());
    }

  @Override
  public void periodic() {
    // This method will be called once per scheduler 

    //SmartDashboard.putNumber("Flywheel top setpoint", tbhController.getTarget());
    SmartDashboard.putNumber("Flywheel Top Target Pos", topProfiler.getTarget());
    SmartDashboard.putNumber("Flywheel Top Pos", getTopPos());
    SmartDashboard.putNumber("Flywheel Top Target Vel", topProfiler.getTargetVel());
    SmartDashboard.putNumber("Flywheel Top Vel", getTopSpeed());
    SmartDashboard.putNumber("Fylwheel Top PWM", topSpeed);
    SmartDashboard.putNumber("Flywheel Bottom Target Pos", bottomProfiler.getTarget());
    SmartDashboard.putNumber("Flywheel Bottom Pos", getBottomPos());
    SmartDashboard.putNumber("Flywheel Bottom Target Vel", bottomProfiler.getTargetVel());
    SmartDashboard.putNumber("Flywheel Bottom Vel", getBottomSpeed());
    SmartDashboard.putNumber("Fylwheel Bottom PWM", bottomSpeed);
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
    this.topSpeed = topSpeed;
    this.bottomSpeed = bottomSpeed;
    topMotor.set(ControlMode.PercentOutput, topSpeed);
    bottomMotor.set(ControlMode.PercentOutput, bottomSpeed);
  }

  public double getTopPos() {
    return topMotor.getSelectedSensorPosition();
  }

  public double getBottomPos() {
    return bottomMotor.getSelectedSensorPosition();
  }

  public double getTopSpeed() {
    return topMotor.getSelectedSensorVelocity();
  }

  public double getBottomSpeed() {
    return bottomMotor.getSelectedSensorVelocity();
  }

  public double getTopTarget() {
    //return tbhController.getTarget();
    return topProfiler.getTarget();
  }

  public double getBottomTarget() {
    //return tbhController.getTarget();
    return topProfiler.getTarget();
  }

  public void setTargets(double topTarget, double bottomTarget) {
    topProfiler.setTarget(topTarget);
    bottomProfiler.setTarget(bottomTarget);
  }
  
  public void setTargetsRelative(double topTarget, double bottomTarget) {
    topProfiler.setTargetRelative(topTarget);
    bottomProfiler.setTargetRelative(bottomTarget);
  }

  public void calculate() {
    move(topProfiler.calculate(getTopPos()), bottomProfiler.calculate(getBottomPos()));
  }

  public boolean atTargets() {
    return topProfiler.atTarget() && bottomProfiler.atTarget();
  }

  public void init() {
    topProfiler.init(getTopPos());
    bottomProfiler.init(getTopPos());
  }

  public void setTopConstants(double kP, double kI, double kD, double kFv, double kFa) {
    topProfiler.setP(kP);
    topProfiler.setI(kI);
    topProfiler.setD(kD);
    topProfiler.setVelocityFeedforward(kFv);
    topProfiler.setAccelFeedforward(kFa);
  }

  public void setBottomConstants(double kP, double kI, double kD, double kFv, double kFa) {
    bottomProfiler.setP(kP);
    bottomProfiler.setI(kI);
    bottomProfiler.setD(kD);
    bottomProfiler.setVelocityFeedforward(kFv);
    bottomProfiler.setAccelFeedforward(kFa);
  }
}