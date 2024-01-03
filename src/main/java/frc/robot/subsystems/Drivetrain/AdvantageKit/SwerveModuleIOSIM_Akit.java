// Copyright 2021-2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.Drivetrain.AdvantageKit;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.libs.BaseMechanism.SimMotorMechanism;

/**
 * Physics sim implementation of module IO.
 *
 * <p>Uses two flywheel sims for the drive and turn motors, with the absolute position initialized
 * to a random value. The flywheel sims are not physically accurate, but provide a decent
 * approximation for the behavior of the module.
 */
public class SwerveModuleIOSIM_Akit implements SwerveModuleIO {
  private static final double LOOP_PERIOD_SECS = 0.02;

  SwerveModuleIOInputs cachedInputs = new SwerveModuleIOInputs();

  private DCMotorSim driveSim = new DCMotorSim(DCMotor.getNEO(1), 6.75, 0.025);
  SimMotorMechanism drive_simMech = new SimMotorMechanism(driveSim);
  private DCMotorSim turnSim = new DCMotorSim(DCMotor.getNEO(1), 150.0 / 7.0, 0.004);
  SimMotorMechanism turn_simMech = new SimMotorMechanism(turnSim);

  private final Rotation2d turnAbsoluteInitPosition = new Rotation2d(Math.random() * 2.0 * Math.PI);
  private double driveAppliedVolts = 0.0;
  private double turnAppliedVolts = 0.0;

  public SwerveModuleIOSIM_Akit() {

    drive_simMech.config_kV(0.13);
    drive_simMech.config_kS(0.0);
    drive_simMech.config_kP(0.1);
    drive_simMech.config_kI(0.0);
    drive_simMech.config_kD(0.0);

    turn_simMech.config_kP(0.1);
    turn_simMech.config_kI(0.0);
    turn_simMech.config_kD(0.0);
  }
  @Override
  public void updateInputs(SwerveModuleIOInputs inputs) {
    drive_simMech.updateSim();
    turn_simMech.updateSim();

    inputs.drivePositionMeters = driveSim.getAngularPositionRad();
    inputs.driveVelocityMetersPerSec = driveSim.getAngularVelocityRadPerSec();
    inputs.driveAppliedVolts = 0;
    inputs.driveCurrentAmps = driveSim.getCurrentDrawAmps();
    inputs.driveTemperatureCelsius = 0;

    inputs.turnAbsolutePosition = turnSim.getAngularPositionRad();
    inputs.turnVelocityRadPerSec = turnSim.getAngularVelocityRadPerSec();
    inputs.turnAppliedVolts = 0;
    inputs.turnCurrentAmps = turnSim.getCurrentDrawAmps();
    inputs.turnTemperatureCelsius = 0;


    cachedInputs = inputs;
  }

  @Override
  public SwerveModuleIOInputs getInputs() {
    return cachedInputs;
  }


  @Override
  public void setSpeed(double speed) {
    drive_simMech.setBasicVelocity(speed);
  }

  @Override
  public void setAngle(double angle) {
    turn_simMech.setRacePIDPosition(0);

  }
  @Override
  public SwerveModuleState optimize(SwerveModuleState swerveModuleState) {
    return SwerveModuleState.optimize(swerveModuleState, new Rotation2d(cachedInputs.turnAbsolutePosition));
  }

  @Override
  public void stopDrive() {
    drive_simMech.setVoltage(0);

  }

  @Override
  public void stopAngle() {
    turn_simMech.setVoltage(0);
  }

  @Override
  public void tuneDrive() {
    drive_simMech.runPIDtuner();

  }

  @Override
  public void tuneAngle() {
    turn_simMech.runPIDtuner();
  }
}
