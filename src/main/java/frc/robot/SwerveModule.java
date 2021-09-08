// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.util.Units;

public class SwerveModule {
  private static final double kWheelRadius = Units.inchesToMeters(2);
  private static final int kEncoderResolution = 2048;

  private static final double kModuleMaxAngularVelocity = Drivetrain.kMaxAngularSpeed;
  private static final double kModuleMaxAngularAcceleration = 2 * Math.PI; // radians per second squared

  private final TalonFX m_driveMotor;
  private final TalonFX m_turningMotor;


  private final PIDController m_drivePIDController = new PIDController(1, 0, 0);

  private final ProfiledPIDController m_turningPIDController = new ProfiledPIDController(1, 0, 0,
      new TrapezoidProfile.Constraints(kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration));

  // Gains are for example purposes only - must be determined for your own robot!
  private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(1, 3);
  private final SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(1, 0.5);

  /**
   * Constructs a SwerveModule.
   *
   * @param driveMotorChannel   ID for the drive motor.
   * @param turningMotorChannel ID for the turning motor.
   */
  public SwerveModule(int driveMotorChannel, int turningMotorChannel) {
    m_driveMotor = new TalonFX(driveMotorChannel);
    m_turningMotor = new TalonFX(turningMotorChannel);

    

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveMotorEncoder(), new Rotation2d(getTurningMotorEncoder()));
  }

  private double getDriveMotorEncoder() {
    return
    m_driveMotor.getSelectedSensorVelocity()*10/kEncoderResolution*Math.PI*kWheelRadius*2;
  }

  private double getTurningMotorEncoder() {
    return
    m_turningMotor.getSelectedSensorVelocity()*10/kEncoderResolution*2*Math.PI;
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state = SwerveModuleState.optimize(desiredState, new Rotation2d(getTurningMotorEncoder()));

    // Calculate the drive output from the drive PID controller.
    final double driveOutput = m_drivePIDController.calculate(getDriveMotorEncoder(), state.speedMetersPerSecond);

    final double driveFeedforward = m_driveFeedforward.calculate(state.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.
    final double turnOutput = m_turningPIDController.calculate(getTurningMotorEncoder(), state.angle.getRadians());

    final double turnFeedforward = m_turnFeedforward.calculate(m_turningPIDController.getSetpoint().velocity);

    m_driveMotor.set(TalonFXControlMode.Current, driveOutput + driveFeedforward);
    m_turningMotor.set(TalonFXControlMode.Current, turnOutput + turnFeedforward);
  }
}
