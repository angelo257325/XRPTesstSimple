// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*
 * Code made by Angelo Morales 
 * Date: 10/18/23
 * Contact Info:
 * Email: amorales@firstnevada.org
 * Phone: 702-917-2215
 * Versions:
 * WPILIB - 2024.1.1 Beta 1
 * XRP Firmware - 0.5.1
 */

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.simulation.XboxControllerSim;
import edu.wpi.first.wpilibj.xrp.XRPMotor;
import edu.wpi.first.wpilibj.xrp.XRPServo;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.MotorSafety;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */

public class Robot extends TimedRobot {

//Setting Up Motors and Differential Drive
public DifferentialDrive m_myRobot;
public XRPMotor m_lefMotor;
public XRPMotor m_rightMotor;

//Setting Up Servo 
public XRPServo m_arm;
public XRPServo servoArm = new XRPServo(4);
 
//Setting Up Xbox Controller
private final XboxController driveStick = new XboxController(1);
  
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {

  //More Motor Setup    
  XRPMotor m_leftMotor = new XRPMotor(0);
  XRPMotor m_rightMotor = new XRPMotor(1);
  m_rightMotor.setInverted(true);

  //Assigns Motors to Differential Drive
  m_myRobot = new DifferentialDrive(m_leftMotor, m_rightMotor);

  //Motor Safety Might Be Needed if Differential Drive is not outputted enough "Restart Should fix issue" if not uncomment
  //m_myRobot.setSafetyEnabled(false);

  }

  @Override
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {

    //Sets Servo to 0 degrees 
    servoArm.setAngle(0);
  }

  @Override
  public void teleopPeriodic() {

    //Drives Robot Using the Differential class.
    //Robot Drives using both xbox control sticks one is for forward and reverse "Left Stick" and the other is for turning "Right Stick"
    m_myRobot.arcadeDrive(driveStick.getLeftY(), -driveStick.getRightX());

    //Controls Servo Using the left trigger of the xbox controller 
    //Commented to not interfere with the if statement below, to use it uncomment and comment the if statement
    //double armAngle = driveStick.getLeftTriggerAxis()*180;
    //servoArm.setAngle(armAngle);

    //Controls the servo using the A and B buttons. Currently the A button moves the servo to about 90 degrees position and the B button moves the servo to 180 degrees position
    if(driveStick.getRawButton(1)){
      servoArm.setAngle(90);
    }else if(driveStick.getRawButton(2)){
      servoArm.setAngle(180);
    }

  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
