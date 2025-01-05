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

import com.revrobotics.spark.SparkMax;
// import com.revrobotics.spark.SparkSim;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;

public class ElevatorSubsystem extends SubsystemBase {

    public SparkMax followerElevator;
    public SparkMax leaderElevator;
    private RelativeEncoder leaderEncoder;
    private RelativeEncoder followerEncoder;
    public SparkClosedLoopController  leaderPIDController;
    public SparkClosedLoopController  followerPIDController;
    public SparkMaxSim leaderElevatorSim;
    public SparkMaxSim followerElevatorSim;
    public SparkRelativeEncoderSim leaderEncoderSim;
    public SparkRelativeEncoderSim followerEncoderSim;
    private double kLeaderP = 0.0005, kLeaderI = 0.0, kLeaderD = 0.0;
    private double kFollowerP = 0.0005, kFollowerI = 0.0, kFollowerD = 0.0;
    private double kLeaderFF = 0.0005, kFollowerFF = 0.0005, kFeederFF = 0.0005;
    private double kLeaderOutputMin = -1.0, kFollowerOutputMin = -1.0, kFeederOutputMin = -1.0;
    private double kLeaderOutputMax = 1.0, kFollowerOutputMax = 1.0, kFeederOutputMax = 1.0;
    private double kLeaderMaxRPM = 5676, kFollowerMaxRPM = 5676, kFeederMaxRPM = 5676;
    private double kLeaderMaxAccel = 10000, kFollowerMaxAccel = 10000, kFeederMaxAccel = 10000;
    

    public ElevatorSubsystem() {
        leaderElevator = new SparkMax(21, MotorType.kBrushless);
        followerElevator = new SparkMax(22, MotorType.kBrushless);

        SparkMaxConfig leaderConfig = new SparkMaxConfig();
        SparkMaxConfig followerConfig = new SparkMaxConfig();
        SoftLimitConfig leaderSoftLimit = new SoftLimitConfig();
        SoftLimitConfig followerSoftLimit = new SoftLimitConfig();

        leaderPIDController = leaderElevator.getClosedLoopController();
        followerPIDController = followerElevator.getClosedLoopController();

        leaderEncoder = leaderElevator.getEncoder();
        followerEncoder = followerElevator.getEncoder();

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
        leaderElevator.configure(leaderConfig,ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        //TODO: Add soft limits
        leaderSoftLimit
        .forwardSoftLimit(1) 
        .reverseSoftLimit(1)
        .apply(leaderSoftLimit);

        followerConfig
            .follow(leaderElevator, true)
            .voltageCompensation(12.0)
            .smartCurrentLimit(80)
            .idleMode(IdleMode.kBrake)
            .closedLoop
                .pidf(kFollowerP, kFollowerI, kFollowerD, kFollowerFF)
                .outputRange(kFollowerOutputMin, kFollowerOutputMax)
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .maxMotion
                    .maxAcceleration(kFollowerMaxAccel)
                    .maxVelocity(kFollowerMaxRPM);
        followerElevator.configure(followerConfig,ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        //TODO: Add soft limits
        followerSoftLimit
        .forwardSoftLimit(1) 
        .reverseSoftLimit(1)
        .apply(leaderSoftLimit);

        // Add motors to the simulation
        if (Robot.isSimulation()) {
            leaderElevatorSim = new SparkMaxSim(leaderElevator, DCMotor.getNEO(1));
            followerElevatorSim = new SparkMaxSim(followerElevator, DCMotor.getNEO(1));
            leaderEncoderSim = new SparkRelativeEncoderSim(leaderElevator);
            followerEncoderSim = new SparkRelativeEncoderSim(followerElevator);
            // leaderElevatorSim.setVelocity(0);
            // followerElevatorSim.setVelocity(0);
            // feederLauncherSim.setVelocity(0);
        }
    }
    
    // // An accessor method to set the speed (technically the output percentage) of the launch wheel
    public void setElevatorHeight(double pos) {
        // leaderElevatorL.set(speed);
        leaderPIDController.setReference(pos, SparkMax.ControlType.kMAXMotionPositionControl);
        if (Robot.isSimulation()) {
            // leaderElevatorSim.setVelocity(speed);
            // followerElevatorSim.setVelocity(speed);
            followerPIDController.setReference(pos, SparkMax.ControlType.kMAXMotionPositionControl);
        }
    }
    
    public Command startPosCmd() {
    return this.startEnd(
        () -> {
            // leaderElevatorL.set(-0.25);
            // feederLauncher.set(-0.25);
            //leaderPIDController.setReference(Constants.ElevatorConstants.START_POSE, SparkMax.ControlType.kMAXMotionPositionControl);
            
            setElevatorHeight(Constants.ElevatorConstants.START_POSE);
            if (Robot.isSimulation()) {
                // leaderElevatorSim.setVelocity(-1000);
                // followerElevatorSim.setVelocity(-1000);
                // feederLauncherSim.setVelocity(-1000);
                //followerPIDController.setReference(Constants.ElevatorConstants.START_POSE, SparkMax.ControlType.kMAXMotionPositionControl);

                setElevatorHeight(Constants.ElevatorConstants.START_POSE);
            }
        },
        () -> {
            // leaderElevatorL.set(0);
            // feederLauncher.set(0);
            // leaderPIDController.setReference(Constants.ElevatorConstants.START_POSE, SparkMax.ControlType.kMAXMotionVelocityControl);
            setElevatorHeight(Constants.ElevatorConstants.START_POSE);
            if (Robot.isSimulation()) {
                // leaderElevatorSim.setVelocity(0);
                // followerElevatorSim.setVelocity(0);
                // feederLauncherSim.setVelocity(0);
                // followerPIDController.setReference(Constants.ElevatorConstants.START_POSE, SparkMax.ControlType.kMAXMotionPositionControl);
                setElevatorHeight(Constants.ElevatorConstants.START_POSE);
            }
        });
    }

    public Command shortPosCmd() {
    return this.startEnd(
        () -> {
            // leaderElevatorL.set(0.10);
            // feederLauncher.set(0.10);
            // leaderPIDController.setReference(Constants.ElevatorConstants.SHORT_POSE, SparkMax.ControlType.kMAXMotionPositionControl);

            setElevatorHeight(Constants.ElevatorConstants.SHORT_POSE);
            if (Robot.isSimulation()) {
                // leaderElevatorSim.setVelocity(500);
                // followerElevatorSim.setVelocity(500);
                // feederLauncherSim.setVelocity(500);
                // followerPIDController.setReference(Constants.ElevatorConstants.SHORT_POSE, SparkMax.ControlType.kMAXMotionPositionControl);
                setElevatorHeight(Constants.ElevatorConstants.SHORT_POSE);
            }
        },
        () -> {
            // leaderElevatorL.set(0);
            // feederLauncher.set(0);
            // leaderPIDController.setReference(Constants.ElevatorConstants.SHORT_POSE, SparkMax.ControlType.kMAXMotionPositionControl);

            setElevatorHeight(Constants.ElevatorConstants.SHORT_POSE);
            if (Robot.isSimulation()) {
                // leaderElevatorSim.setVelocity(0);
                // followerElevatorSim.setVelocity(0);
                // feederLauncherSim.setVelocity(0);
                // followerPIDController.setReference(Constants.ElevatorConstants.MIDDLE_POSE, SparkMax.ControlType.kVelocity);
                setElevatorHeight(Constants.ElevatorConstants.SHORT_POSE);
            }
        });
    }

    public Command middlePosCmd() {
        return this.startEnd(
            () -> {
                // leaderElevatorL.set(0.10);
                // feederLauncher.set(0.10);
                // leaderPIDController.setReference(Constants.ElevatorConstants.MIDDLE_POSE, SparkMax.ControlType.kMAXMotionPositionControl);
                setElevatorHeight(Constants.ElevatorConstants.MIDDLE_POSE);
                if (Robot.isSimulation()) {
                    // leaderElevatorSim.setVelocity(500);
                    // followerElevatorSim.setVelocity(500);
                    // feederLauncherSim.setVelocity(500);
                    setElevatorHeight(Constants.ElevatorConstants.MIDDLE_POSE);
                }
            },
            () -> {
                // leaderElevatorL.set(0);
                // feederLauncher.set(0);
                // leaderPIDController.setReference(Constants.ElevatorConstants.MIDDLE_POSE, SparkMax.ControlType.kMAXMotionPositionControl);
                setElevatorHeight(Constants.ElevatorConstants.MIDDLE_POSE);
                if (Robot.isSimulation()) {
                    // leaderElevatorSim.setVelocity(0);
                    // followerElevatorSim.setVelocity(0);
                    // feederLauncherSim.setVelocity(0);
                    setElevatorHeight(Constants.ElevatorConstants.MIDDLE_POSE);
                }
            });
        }

        public Command tallPosCmd() {
            return this.startEnd(
                () -> {
                    // leaderElevatorL.set(0.10);
                    // feederLauncher.set(0.10);
                    // leaderPIDController.setReference(Constants.ElevatorConstants.TALL_POSE, SparkMax.ControlType.kMAXMotionPositionControl);
                    setElevatorHeight(Constants.ElevatorConstants.TALL_POSE);
                    if (Robot.isSimulation()) {
                        // leaderElevatorSim.setVelocity(500);
                        // followerElevatorSim.setVelocity(500);
                        // feederLauncherSim.setVelocity(500);
                        setElevatorHeight(Constants.ElevatorConstants.TALL_POSE);
                    }
                },
                () -> {
                    // leaderElevatorL.set(0);
                    // feederLauncher.set(0);
                    // leaderPIDController.setReference(Constants.ElevatorConstants.TALL_POSE, SparkMax.ControlType.kMAXMotionPositionControl);
                    setElevatorHeight(Constants.ElevatorConstants.TALL_POSE);
                    if (Robot.isSimulation()) {
                        // leaderElevatorSim.setVelocity(0);
                        // followerElevatorSim.setVelocity(0);
                        // feederLauncherSim.setVelocity(0);
                        setElevatorHeight(Constants.ElevatorConstants.TALL_POSE);
                    }
                });
            }
    

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
        if (Robot.isSimulation()) {
            leaderEncoderSim.setPosition(leaderElevatorSim.getPosition());
            followerEncoderSim.setPosition(followerElevatorSim.getPosition());
            leaderElevatorSim.iterate(leaderEncoderSim.getPosition(), leaderElevatorSim.getBusVoltage(),.005);
            followerElevatorSim.iterate(followerEncoderSim.getPosition(), followerElevatorSim.getBusVoltage(),.005);
        }
    }
    
    @Override
    public void periodic() {
    // This method will be called once per scheduler run
    if (Robot.isSimulation()) {
        SmartDashboard.putNumber("Lead Launch Wheel Speed (RPM)", leaderEncoderSim.getPosition());
        SmartDashboard.putNumber("Follower Launch Wheel Speed (RPM)", followerEncoderSim.getPosition());
    } else {
        SmartDashboard.putNumber("Lead Launch Wheel Speed (RPM)", leaderEncoder.getPosition());
        SmartDashboard.putNumber("Follower Launch Wheel Speed (RPM)", followerEncoder.getPosition());
    }
    }
}