/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.*;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.controller.PIDController;
public class Neo extends SubsystemBase {

    //public static final int KTopSparkMax = 11;
    //public static final int KBottomSparkMax = 10;
    public static final int KTopTalon = 4;
    public static final int KBottomTalon = 2;

    public static double topMotorSpeed;
    public static double bottomMotorSpeed;

    public static PIDController flywheelTopPID;
    public static PIDController flywheelBottomPID; 
    public static double setPoint;

    //private CANSparkMax TopMotor; 
	  //private CANSparkMax BottomMotor; 
    private TalonSRX TopMotor;
    private TalonSRX BottomMotor;

    private double topSpeed = 0;
    private double bottomSpeed = 0;
    private double topAccel = 0;
    private double bottomAccel = 0;

    public Neo () {
		  // TopMotor = new CANSparkMax(TopSparkMax, CANSparkMaxLowLevel.MotorType.kBrushless);
      // BottomMotor = new CANSparkMax(BottomSparkMax, CANSparkMaxLowLevel.MotorType.kBrushless);
      TopMotor = new TalonSRX(KTopTalon);
      BottomMotor = new TalonSRX(KBottomTalon);

      TopMotor.setInverted(true);
      BottomMotor.setInverted(false);

      TopMotor.setSensorPhase(true);

      flywheelTopPID = new PIDController(0.0001, 0, 0);
      flywheelBottomPID = new PIDController(0.00001, 0, 0);

      flywheelTopPID.enableContinuousInput(-40000, 40000);
      flywheelTopPID.disableContinuousInput();

      flywheelTopPID.reset();
      //flywheelTopPID.
    }

  @Override
  public void periodic() {
    // This method will be called once per scheduler 
    SmartDashboard.putNumber("Flywheel top speed", getTopSpeed()); 
    SmartDashboard.putNumber("Flywheel bottom speed", getBottomSpeed());
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

    TopMotor.set(ControlMode.PercentOutput, topSpeed);
    BottomMotor.set(ControlMode.PercentOutput, bottomSpeed);
    SmartDashboard.putNumber("Top Flywheel VOLTAGE", topSpeed);
  }

  public double getTopSpeed() {
    return TopMotor.getSelectedSensorVelocity();
  }

  public double getBottomSpeed() {
    return BottomMotor.getSelectedSensorVelocity();
  }

  public void setTopSetpoint (double setpoint){
    SmartDashboard.putNumber("Top flywheel setpoint", setpoint);
    flywheelTopPID.setSetpoint(setpoint);
  }

  public void setBottomSetpoint (double setpoint){
    flywheelBottomPID.setSetpoint(setpoint);
  }

  public double getTopSetpoint() {
    return flywheelTopPID.getSetpoint();
  }

  public double getBottomSetpoint() {
    return flywheelBottomPID.getSetpoint();
  }

  public void calculate() {
    //flywheelBottomPID.calculate(getBottomSpeed())
    topAccel = flywheelTopPID.calculate(getTopSpeed());
    SmartDashboard.putNumber("Top flywheel accel", topAccel);
    bottomAccel = flywheelBottomPID.calculate(getTopSpeed());

    move(topAccel + topSpeed, bottomSpeed);

    SmartDashboard.putNumber("Top Flywheel error", flywheelTopPID.getPositionError());
  }
}
