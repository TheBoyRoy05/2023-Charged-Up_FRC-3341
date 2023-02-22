// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import javax.swing.text.StyleContext.SmallAttributeSet;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;

public class AutoBalance extends CommandBase {
  /** Creates a new AutoBalance. */
  DriveTrain dt;
  //double maxPower = 0.5; //if the robot was vertical(theoretically) the talons would go at this power 
  double baseSpeed;
  double angleThreshhold;
  double previousAngle;
  double currentAngle;

  PIDController pid;
  PIDController yawPID;

  public AutoBalance( DriveTrain dt) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.dt = dt;
    baseSpeed = 0.33;
    pid = new PIDController(0.5, 0.4, 0.1);
    yawPID = new PIDController(0.02, 0, 0);
    angleThreshhold = Constants.OperatorConstants.angleThreshhold;
    addRequirements(dt);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    dt.resetEncoders();
    pid.setSetpoint(Constants.OperatorConstants.balanceDistance);
    yawPID.setSetpoint(dt.getAngle());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = pid.calculate(dt.getDisplacement());
    double turningSpeed = yawPID.calculate(dt.getAngle());
    SmartDashboard.putNumber("Docking Speed: ", speed);

    dt.tankDrive(speed + turningSpeed, speed - turningSpeed);

    if(0.005 >= Math.abs(Constants.OperatorConstants.balanceDistance - dt.getDisplacement()) 
            && Constants.OperatorConstants.angleThreshhold <= dt.getYAngle()){
        Constants.OperatorConstants.balanceDistance -= 0.01 * Math.abs(dt.getYAngle())/dt.getYAngle();
    } 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    dt.brake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //return .03 >= Math.abs(Constants.OperatorConstants.balanceDistance - dt.getDisplacement());
    return false;
    //return 3 >= Math.abs(dt.getYAngle());
  }
}
