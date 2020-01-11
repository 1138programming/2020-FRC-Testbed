/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;


import com.revrobotics.*;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Neo extends SubsystemBase {

    public static final int KSparkMax1 = 10; 
	public static final int KSparkMax2 = 2; 

    private CANSparkMax NeoMotor1; 
	private CANSparkMax NeoMotor2; 

    public Neo () {
		NeoMotor1 = new CANSparkMax(KSparkMax1, CANSparkMaxLowLevel.MotorType.kBrushless);
		NeoMotor2 = new CANSparkMax(KSparkMax2, MotorType.kBrushless);
        NeoMotor2.setInverted(true);
        NeoMotor2.follow(NeoMotor1);
    }

  @Override
  public void periodic() {
    // This method will be called once per scheduler 
  }

  public void neoMotorMove(double speed) {
		NeoMotor1.set(speed);
	}

  public void neoMotorStop() {
		NeoMotor1.set(0);
	}
}
