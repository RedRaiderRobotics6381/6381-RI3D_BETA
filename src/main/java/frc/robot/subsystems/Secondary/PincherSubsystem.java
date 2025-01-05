// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Secondary;

import com.revrobotics.servohub.ServoHub;
import com.revrobotics.servohub.config.ServoHubConfig;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class PincherSubsystem extends SubsystemBase {
  /** Creates a new PincherSubsystem. */
  public Servo launchServo;
  public Servo pincherServo;
  
  public PincherSubsystem() {
    launchServo = new Servo(Constants.ServoConstants.PINCHER_SERVO_PORT_1);
    pincherServo = new Servo(Constants.ServoConstants.PINCHER_SERVO_PORT_2);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }
  public void setPinchAngle(double angle){ {
    pincherServo.setAngle(angle);
  }
 }
 public Command ClosedCmd() {
  return this.runOnce(
      () -> {
          // rotateMotorL.set(-0.25);
          // feederLauncher.set(-0.25);
          //rotatePIDController.setReference(-1000, SparkMax.ControlType.kMAXMotionPositionControl);
          setPinchAngle(Constants.ServoConstants.CLOSED_ANGLE);
      }
    );
  }
  
  public Command OpenCmd() {
    return this.runOnce(
        () -> {
            // rotateMotorL.set(-0.25);
            // feederLauncher.set(-0.25);
            //rotatePIDController.setReference(-1000, SparkMax.ControlType.kMAXMotionPositionControl);
            setPinchAngle(Constants.ServoConstants.OPEN_ANGLE);
        }
      );
    }
}



