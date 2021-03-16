// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.XboxControllerSim;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;

//Limelight importations :DDDDDDDDDDDDDDD
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends TimedRobot {
  VictorSP frontLeftMotor = new VictorSP(0);
  VictorSP frontRightMotor = new VictorSP(6);
  VictorSP backLeftMotor = new VictorSP(1);
  VictorSP backRightMotor = new VictorSP(7);
  XboxController xbox = new XboxController(0);
  Spark ballIntake = new Spark(4);

  SpeedControllerGroup leftMotors = new SpeedControllerGroup(frontLeftMotor, backLeftMotor);
  SpeedControllerGroup rightMotors = new SpeedControllerGroup(frontRightMotor, backRightMotor);
    
  DifferentialDrive drive = new DifferentialDrive(leftMotors, rightMotors);
  private final Joystick m_stick = new Joystick(0);
  private final Timer m_timer = new Timer();
  
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    /*
      UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
      camera.setResolution(320, 240);
      camera.setFPS(30);
      */
      //Only un-comment if using a usb webcam rather than the limelight
  }

  /** This function is run once each time the robot enters autonomous mode. */
  @Override
  public void autonomousInit() {
    m_timer.reset();
    m_timer.start();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    // Drive for 2 seconds
    if (m_timer.get() < 1.0) {
      drive.arcadeDrive(0.5, 0.0); // drive forwards half speed
    } else if ((m_timer.get() > 1.0) && ((m_timer.get() < 2.0))) {
      drive.arcadeDrive(0.5, 0.5); // turn (hopefully slowly)
    } else {
      drive.arcadeDrive(0.0, 0.0); // stop robot
    }
  }

  /** This function is called once each time the robot enters teleoperated mode. */
  @Override
  public void teleopInit() {
    
  }

  /** This function is called periodically during teleoperated mode. */
  @Override
  public void teleopPeriodic() {
    ballDrive(true);
    if(xbox.getXButton() == true){
        goalLock();
    }
  }

  /** This function is called once each time the robot enters test mode. */
  @Override
  public void testInit() {

  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    
  }

  public void goalLock(){
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");
    double x = tx.getDouble(0.0);
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");

    //read values periodically
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);

    //post to smart dashboard periodically
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);

    if(x < -0.5){
      drive.tankDrive(minMax(0.5*x/5.0, -0.5, 0.5), minMax(-0.5*x/5.0, -0.5, 0.5));
    }else if(x > 0.5){
      drive.tankDrive(minMax(0.5*x/5.0, -0.5, 0.5), minMax(-0.5*x/5.0, -0.5, 0.5));
    }else{
      drive.tankDrive(0.0, 0.0);
    }
  }

  public double minMax(double num, double min, double max){
    if(num < min){
      num = min;
    }else if(num > max){
      num = max;
    }
    return num;
  }

  public void ballDrive(boolean yPress){
    double turn;
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");
    double x = tx.getDouble(0.0);
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");

    //read values periodically
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);

    //post to smart dashboard periodically
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);

    if(xbox.getYButton() == true){
      //write swerve code here (yung goku)
      turn = x/30;
    }else{
      turn = m_stick.getX();
    }
    drive.arcadeDrive(-m_stick.getY()/(1.5 - xbox.getTriggerAxis(Hand.kLeft)), turn);
    ballIntake.set(-xbox.getTriggerAxis(Hand.kRight));
  }
}
