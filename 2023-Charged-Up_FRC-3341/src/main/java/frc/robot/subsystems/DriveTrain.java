// Based upon 2021's Competition Season DriveTrain code

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.AnalogAccelerometer;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants;
import frc.robot.Robot;

import com.kauailabs.navx.frc.AHRS;
//comment


public class DriveTrain extends SubsystemBase 
{
  /** Creates a new ExampleSubsystem. */
  private final WPI_TalonSRX leftDriveTalon;
  private final WPI_TalonSRX rightDriveTalon;
  private final VictorSPX _leftDriveVictor;
  private final VictorSPX _rightDriveVictor;
  private final AHRS navX;
  public boolean logOverride = false;
  public DoubleLogEntry anglelog;

  
  public DriveTrain() 
  {
    DataLogManager.start();
    DataLog log = DataLogManager.getLog();
    anglelog = new DoubleLogEntry(log, "/my/double");
    
    leftDriveTalon = new WPI_TalonSRX(Constants.OperatorConstants.LeftDriveTalonPort);
    rightDriveTalon = new WPI_TalonSRX(Constants.OperatorConstants.RightDriveTalonPort);
    _leftDriveVictor = new VictorSPX(Constants.OperatorConstants.LeftDriveVictorPort);
    _rightDriveVictor = new VictorSPX(Constants.OperatorConstants.RightDriveVictorPort);
    navX = new AHRS(SPI.Port.kMXP);

    leftDriveTalon.configFactoryDefault();
    leftDriveTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
    rightDriveTalon.configFactoryDefault();
    rightDriveTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);

    _leftDriveVictor.follow(leftDriveTalon);
    _rightDriveVictor.follow(rightDriveTalon);   
  
    leftDriveTalon.setNeutralMode(NeutralMode.Coast);
    rightDriveTalon.setNeutralMode(NeutralMode.Coast);

    leftDriveTalon.setInverted(false);
    rightDriveTalon.setInverted(true);
    _leftDriveVictor.setInverted(InvertType.FollowMaster);
    _rightDriveVictor.setInverted(InvertType.FollowMaster);

    leftDriveTalon.setSensorPhase(true);
    rightDriveTalon.setSensorPhase(true);
    
    // Week 4 Motion Magic
   leftDriveTalon.config_kP(0, 2.5, 10);
   leftDriveTalon.config_kI(0, 0, 10);
   leftDriveTalon.config_kD(0, 0, 10);
   leftDriveTalon.configMotionAcceleration(328, 10);
   leftDriveTalon.configMotionCruiseVelocity(400, 10);
   // If Left Velocity is 200
   // then Left Accel 120

   rightDriveTalon.config_kP(0, 2.5, 10);
   rightDriveTalon.config_kI(0, 0, 10);
   rightDriveTalon.config_kD(0, 0, 10);
   rightDriveTalon.configMotionAcceleration(2000, 10);
   rightDriveTalon.configMotionCruiseVelocity(386, 10);
  }

  public void brake(){
    leftDriveTalon.setNeutralMode(NeutralMode.Brake);
    rightDriveTalon.setNeutralMode(NeutralMode.Brake);
  }

  public void coast(){
    leftDriveTalon.setNeutralMode(NeutralMode.Coast);
    rightDriveTalon.setNeutralMode(NeutralMode.Coast);
  }

  public void setPowerLimits(double limit){
    leftDriveTalon.configPeakOutputForward(limit);
    leftDriveTalon.configPeakOutputReverse(limit);
    rightDriveTalon.configPeakOutputForward(limit);
    rightDriveTalon.configPeakOutputReverse(limit);

  }

  public void magicDrive(double displacement){
    leftDriveTalon.set(ControlMode.MotionMagic, Constants.OperatorConstants.tickstoMeters*displacement);
    rightDriveTalon.set(ControlMode.MotionMagic, Constants.OperatorConstants.tickstoMeters*displacement);
  }

  public void tankDrive(double leftSpeed, double rightSpeed) {
    leftDriveTalon.set(leftSpeed);
    rightDriveTalon.set(rightSpeed);
    SmartDashboard.putNumber("leftspeed", leftSpeed);
    SmartDashboard.putNumber("rightspeed", rightSpeed);
  }

  
  public void resetEncoders() {
    leftDriveTalon.setSelectedSensorPosition(0);
    rightDriveTalon.setSelectedSensorPosition(0);
  }

  public void resetNavX(){
    navX.reset();
  }
  public double getTicksLeft() {
    return leftDriveTalon.getSelectedSensorPosition();
  }

  public double getDisplacementLeft(){
    return (getTicksLeft() * Constants.OperatorConstants.tickstoMeters);
  }

  public double getTicksRight() {
    return rightDriveTalon.getSelectedSensorPosition();
  }

  public double getDisplacementRight(){
    return (getTicksRight() * Constants.OperatorConstants.tickstoMeters);
  }

  public double getDisplacement(){
    return (getDisplacementLeft() + getDisplacementRight()) / 2.0;
  }

  public double getYAngle(){
    return navX.getRoll();
  }

  public double getAngle(){
    return navX.getAngle();
  }


  @Override
  public void periodic() {
    tankDrive(RobotContainer.getJoy1().getY()*0.7, RobotContainer.getJoy1().getThrottle()*0.7);
    if (RobotContainer.getJoy1().getRawButtonReleased(12)){
      logOverride = !logOverride;
    }
    if (logOverride){
      anglelog.append(getAngle());
    }
    SmartDashboard.putNumber("Displacement: ", getDisplacement());
    SmartDashboard.putNumber("Angle: ", getYAngle());
  }

  @Override
  public void simulationPeriodic() 
  {
    // This method will be called once per scheduler run during simulation
  }
}