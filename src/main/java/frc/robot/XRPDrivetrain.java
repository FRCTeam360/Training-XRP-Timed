// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.xrp.XRPGyro;
import edu.wpi.first.wpilibj.xrp.XRPMotor;
import edu.wpi.first.wpilibj.xrp.XRPRangefinder;
import edu.wpi.first.wpilibj.xrp.XRPReflectanceSensor;

public class XRPDrivetrain {
  private ShuffleboardTab tab = Shuffleboard.getTab("Drivebase");

  private static final double kGearRatio =
      (30.0 / 14.0) * (28.0 / 16.0) * (36.0 / 9.0) * (26.0 / 8.0); // 48.75:1
  private static final double kCountsPerMotorShaftRev = 12.0;
  private static final double kCountsPerRevolution = kCountsPerMotorShaftRev * kGearRatio; // 585.0
  private static final double kWheelDiameterInch = 2.3622; // 60 mm

  // The XRP has the left and right motors set to
  // channels 0 and 1 respectively
  private final XRPMotor leftMotor = new XRPMotor(0);
  private final XRPMotor rightMotor = new XRPMotor(1);

  // The XRP has onboard encoders that are hardcoded
  // to use DIO pins 4/5 and 6/7 for the left and right
  private final Encoder leftEncoder = new Encoder(4, 5);
  private final Encoder rightEncoder = new Encoder(6, 7);

  private final XRPGyro gyro = new XRPGyro();
  private final XRPRangefinder rangeFinder = new XRPRangefinder();
  private final XRPReflectanceSensor refSensor = new XRPReflectanceSensor();

  private final double kP = 0.0;
  private final double kI = 0.0;
  private final double kD = 0.0;

  private final PIDController pidController = new PIDController(kP, kI, kD);

  private double angleSetPoint = 0.0;

  // Set up the differential drive controller
  private final DifferentialDrive diffDrive =
      new DifferentialDrive(leftMotor::set, rightMotor::set);

  /** Creates a new XRPDrivetrain. */
  public XRPDrivetrain() {
    // Use inches as unit for encoder distances
    leftEncoder.setDistancePerPulse((Math.PI * kWheelDiameterInch) / kCountsPerRevolution);
    rightEncoder.setDistancePerPulse((Math.PI * kWheelDiameterInch) / kCountsPerRevolution);
    resetEncoders();

    // Invert right side since motor is flipped
    rightMotor.setInverted(true);
    setupTab();
  }

  private void setupTab() {
    tab.addDouble("Gyro: Pitch", gyro::getAngleX);
    tab.addDouble("Gyro: Roll", gyro::getAngleY);
    tab.addDouble("Gyro: Yaw", gyro::getAngleZ);

    tab.addDouble("Range Finder: Distance Inches", rangeFinder::getDistanceInches);
    
    tab.addDouble("Reflectance Sensor: Left", refSensor::getLeftReflectanceValue);
    tab.addDouble("Reflectance Sensor: Right", refSensor::getRightReflectanceValue);

    tab.addDouble("Angle Setpoint", () -> this.angleSetPoint);
  }

  public void arcadeDrive(double xaxisSpeed, double zaxisRotate) {
    diffDrive.arcadeDrive(xaxisSpeed, zaxisRotate);
  }

  public void tankDrive(double leftDutyCycle, double rightDutyCycle){

  }

  public void pointAtAngle(double angleSetpoint) {
    double currAngle = gyro.getAngle();
    double output = pidController.calculate(currAngle, angleSetpoint);
    diffDrive.arcadeDrive(0.0, output);
  }

  public void resetEncoders() {
    leftEncoder.reset();
    rightEncoder.reset();
  }

  public double getLeftDistanceInch() {
    return leftEncoder.getDistance();
  }

  public double getRightDistanceInch() {
    return rightEncoder.getDistance();
  }
}
