/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.NeoWithJoysticks; 
import frc.robot.commands.NeoStop;
import frc.robot.subsystems.Neo;
import frc.robot.Robot;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.Joystick;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
   //// CREATING BUTTONS
  // One type of button is a joystick button which is any button on a
  //// joystick.
  // You create one by telling it which joystick it's on and which button
  // number it is.
  //Create Robot

  public static Neo neo = new Neo();

  //Controller Constants 
  public static final int KLogitechDrive = 0;
  public static final int KXboxArms = 1;

  //DeadZone
  public static final double KDeadZoneAxis = 0.2; 
  public static final double KDeadZone = 0.05; 

  //Logitech Button Constants 
  public static final int KButton1 = 1;
	public static final int KButton2 = 2;
	public static final int KButton3 = 3;
	public static final int KButton4 = 4;
	public static final int KButton5 = 5;
	public static final int KButton6 = 6;
	public static final int KButton7 = 7;
  public static final int KButton8 = 8;
  
  private static final int KLeftYAxis = 1;
  private static final int KRightYAxis = 3;

  //Xbox Button Constants 
  public static final int KButtonA = 1;
	public static final int KButtonB = 2;
	public static final int KButtonX = 3;
	public static final int KButtonY = 4;
	public static final int KLeftBumper = 5;
	public static final int KRightBumper = 6;
	public static final int KStartButton = 8;
	public static final int KLeftTrigger = 9;
  public static final int KRightTrigger = 10;

  public Joystick logitech, xbox;
	public JoystickButton btn1, btn2, btn3, btn4, btn5, btn6, btn7, btn8; // Logitech Button
	public JoystickButton btnA, btnB, btnX, btnY, btnLB, btnRB, btnStrt, btnLT, btnRT; // Xbox Buttons

  public RobotContainer() {
    //Default Commands?
    /*base.setDefaultCommand(new RunCommand(() -> {
      SmartDashboard.putBoolean("");
      return;
    }));*/
    //neo.setDefaultCommand(new NeoStop());
    neo.setDefaultCommand(new NeoWithJoysticks());
    //Controllers 
    logitech = new Joystick(KLogitechDrive);
    xbox = new Joystick(KXboxArms);

    //Logitech Buttons
		btn1 = new JoystickButton(logitech, KButton1);
		btn2 = new JoystickButton(logitech, KButton2);
		btn3 = new JoystickButton(logitech, KButton3);
		btn4 = new JoystickButton(logitech, KButton4);
		btn5 = new JoystickButton(logitech, KButton5);
		btn6 = new JoystickButton(logitech, KButton6);
		btn7 = new JoystickButton(logitech, KButton7);
		btn8 = new JoystickButton(logitech, KButton8);

		//XBox Buttons
		btnA = new JoystickButton(xbox, KButtonA);
		btnB = new JoystickButton(xbox, KButtonB);
		btnX = new JoystickButton(xbox, KButtonX);
		btnY = new JoystickButton(xbox, KButtonY);
		btnLB = new JoystickButton(xbox, KLeftBumper);
		btnRB = new JoystickButton(xbox, KRightBumper);
		btnStrt = new JoystickButton(xbox, KStartButton);
		btnLT = new JoystickButton(xbox, KLeftTrigger);
    btnRT = new JoystickButton(xbox, KRightTrigger);
    
    //Button Assigned Commands 
    // EX: btnLT.whenPressed(new ArmUp());

    // Configure the button bindings
    configureButtonBindings();
  }
  /*
  public double getRightAxis() {
    if(logitech.getThrottle() > KDeadZoneAxis || logitech.getThrottle() < -KDeadZoneAxis){
      return -logitech.getThrottle(); 
    }
    else {
      return 0; 
    }
  }
  public double getLeftAxis() {
    if(logitech.getY() > KDeadZoneAxis || logitech.getY() < -KDeadZoneAxis){
      return -logitech.getY(); 
    }
    else {
      return 0; 
    }
  }
  */

  public double getRightAxis() {
    double Y = logitech.getRawAxis(KRightYAxis);
    SmartDashboard.putNumber("Right axis", Y);
    if (Y > KDeadZone || Y < -KDeadZone) {
      return Y;
    } else {
      return 0; 
    }
  }

  public double getLeftAxis() {
    double Y = logitech.getRawAxis(KLeftYAxis);
    SmartDashboard.putNumber("Left axis", Y);
    if (Y > KDeadZone || Y < -KDeadZone) {
      return Y;
    } else {
      return 0; 
    }
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // btn1.whileHeld(new RunCommand(() -> falcon.move(0.5)));
    // btn2.whileHeld(new RunCommand(() -> falcon.move(-0.5)));
    //btn3.whileHeld(new RunCommand(() -> neo.move(0.7, 0.7)));
    //btn4.whileHeld(new RunCommand(() -> neo.move(0.8, 0.8)));
    //btn5.whileHeld(new RunCommand(() -> neo.move(0.9, 0.9)));
    //btn6.whileHeld(new RunCommand(() -> neo.move(1.0, 1.0)));
    //btn3.whileHeld(new RunCommand(() -> neo.move(1.0)));
    //btn4.whileHeld(new RunCommand(() -> neo.move(-1.0)));
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An DriveWithJoysticks will run in autonomous
    return null;
  }
}
