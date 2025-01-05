// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems.Secondary;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.ArmConstants;

import com.revrobotics.spark.SparkMax;
// import com.revrobotics.spark.SparkSim;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;

public class RotateSubsystem extends SubsystemBase {

    public SparkMax rotateMotor;
    private RelativeEncoder rotateEncoder;
    public SparkClosedLoopController  rotatePIDController;
    public SparkMaxSim rotateMotorSim;
    public SparkRelativeEncoderSim rotateEncoderSim;
    private double kLeaderP = 0.0005, kLeaderI = 0.0, kLeaderD = 0.0;
    private double kLeaderFF = 0.0005, kFollowerFF = 0.0005, kFeederFF = 0.0005;
    private double kLeaderOutputMin = -1.0, kFollowerOutputMin = -1.0, kFeederOutputMin = -1.0;
    private double kLeaderOutputMax = 1.0, kFollowerOutputMax = 1.0, kFeederOutputMax = 1.0;
    private double kLeaderMaxRPM = 5676, kFollowerMaxRPM = 5676, kFeederMaxRPM = 5676;
    private double kLeaderMaxAccel = 10000, kFollowerMaxAccel = 10000, kFeederMaxAccel = 10000;
    

    public RotateSubsystem() {
        rotateMotor = new SparkMax(21, MotorType.kBrushless);
        SparkMaxConfig leaderConfig = new SparkMaxConfig();
        SoftLimitConfig leaderSoftLimit = new SoftLimitConfig();

        rotatePIDController = rotateMotor.getClosedLoopController();

        rotateEncoder = rotateMotor.getEncoder();

        leaderConfig
            .inverted(false)
            .voltageCompensation(12.0)
            .smartCurrentLimit(80)
            .idleMode(IdleMode.kBrake)
            .closedLoop
                .pidf(kLeaderP, kLeaderI, kLeaderD, kLeaderFF)
                .outputRange(kLeaderOutputMin, kLeaderOutputMax)
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .maxMotion
                    .maxAcceleration(kLeaderMaxAccel)
                    .maxVelocity(kLeaderMaxRPM);
        rotateMotor.configure(leaderConfig,ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        //TODO: Add soft limits
        leaderSoftLimit
        .forwardSoftLimit(1) 
        .reverseSoftLimit(1)
        .apply(leaderSoftLimit);
            
        // Add motors to the simulation
        if (Robot.isSimulation()) {
            rotateMotorSim = new SparkMaxSim(rotateMotor, DCMotor.getNEO(1));
            rotateEncoderSim = new SparkRelativeEncoderSim(rotateMotor);
            // rotateMotorSim.setVelocity(0);
            // followerLauncherSim.setVelocity(0);
            // feederLauncherSim.setVelocity(0);
        }
    }
    
    // // An accessor method to set the speed (technically the output percentage) of the launch wheel
    public void setArm(double pos) {
        // rotateMotorL.set(speed);
        rotatePIDController.setReference(pos, SparkMax.ControlType.kMAXMotionPositionControl);

    }
    
    public Command ForwardCmd() {
    return this.startEnd(
        () -> {
            // rotateMotorL.set(-0.25);
            // feederLauncher.set(-0.25);
            //rotatePIDController.setReference(-1000, SparkMax.ControlType.kMAXMotionPositionControl);
            setArm(Constants.ArmConstants.ARM_OUT_POSE);

        },
        () -> {
            // rotateMotorL.set(0);
            // feederLauncher.set(0);
            //rotatePIDController.setReference(0, SparkMax.ControlType.kVelocity);
            setArm(Constants.ArmConstants.ARM_OUT_POSE);
        });
    }

    public Command MiddleCmd() {
    return this.startEnd(
        () -> {
            // rotateMotorL.set(0.10);
            // feederLauncher.set(0.10);
            //rotatePIDController.setReference(500, SparkMax.ControlType.kVelocity);
            setArm(Constants.ArmConstants.ARM_MIDDLE_POSE);
        },
        () -> {
            // rotateMotorL.set(0);
            // feederLauncher.set(0);
            // rotatePIDController.setReference(0, SparkMax.ControlType.kVelocity);
            // rotatePIDController.setReference(0, SparkMax.ControlType.kVelocity);
            setArm(Constants.ArmConstants.ARM_MIDDLE_POSE);

        });
    }

    public Command UpCmd() {
      return this.startRun(
          () -> {
              // rotateMotorL.set(0.10);
              // feederLauncher.set(0.10);
              //rotatePIDController.setReference(500, SparkMax.ControlType.kVelocity);
              setArm(Constants.ArmConstants.ARM_UP_POSE);
          },
          () -> {
              // rotateMotorL.set(0);
              // feederLauncher.set(0);
              // rotatePIDController.setReference(0, SparkMax.ControlType.kVelocity);
              // rotatePIDController.setReference(0, SparkMax.ControlType.kVelocity);
              setArm(Constants.ArmConstants.ARM_UP_POSE);
  
          });
      }
    
    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
        if (Robot.isSimulation()) {
            rotateEncoderSim.setPosition(rotateMotorSim.getPosition());
            rotateMotorSim.iterate(rotateEncoderSim.getPosition(), rotateMotorSim.getBusVoltage(),.005);
        }
    }
    
    @Override
    public void periodic() {
    // This method will be called once per scheduler run
    if (Robot.isSimulation()) {
        SmartDashboard.putNumber("Lead Launch Wheel Speed (RPM)", rotateEncoderSim.getPosition());
    } else {
        SmartDashboard.putNumber("Lead Launch Wheel Speed (RPM)", rotateEncoder.getPosition());
    }
    }
}