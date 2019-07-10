package org.minutebots.lib;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;

public class SmartMotor implements SpeedController, Sendable {
	private SpeedController controller;
	private Encoder encoder;
	private String name, subsystem;
	private double max_velocity, P, I, D;
	private double rateConstant;

	public SmartMotor(SpeedController controller, Encoder encoder) {
		this(controller, encoder, "", 0.0, 0.0, 0.0, 0.0, 1);
		throw new Error("Motor instantiated without PIDF values");
	}
	
	public SmartMotor(SpeedController controller, Encoder encoder, double max_velocity, double P, double I, double D) {
		this(controller, encoder, "", max_velocity, P, I, D,1);
	}

	public SmartMotor(SpeedController controller, Encoder encoder, double max_velocity, double P, double I, double D, double rateConstant) {
		this(controller, encoder, "", max_velocity, P, I, D,rateConstant);
	}

	public SmartMotor(SpeedController controller, Encoder encoder, String name, double max_velocity, double P, double I, double D, double rateConstant) {
		this.max_velocity = max_velocity; 	
		this.controller = controller;
		this.encoder = encoder;
		this.name = name;
		this.P = P;
		this.I = I;
		this.D = D;
		this.rateConstant = rateConstant;
	}

	public double getRawVelocity() {
		return encoder.getRate();
	}

	public double getVelocity() { return rateConstant * encoder.getRate(); }
	
	@Override
	public void set(double speed) {
		controller.set(speed);
	}

	@Override
	public void setInverted(boolean isInverted) {
		controller.setInverted(isInverted);
	}

	@Override 
	public boolean getInverted() {
		return controller.getInverted();
	}

	@Override
	public void disable() {
		controller.setInverted(false);
		controller.set(0);
	}

	@Override
	public double get() {
		return controller.get();
	}

	@Override
	public void stopMotor() {
		controller.stopMotor();
	}

	@Override
	public String getName() {
		return name;
	}

	@Override
	public void setName(String name) {
		this.name = name;
	}

	@Override
	public void pidWrite(double output) {
		controller.set(output);
	}

	@Override
	public void setName(String subsystem, String name) {
		this.subsystem = subsystem;
		this.name = name;
	}

	@Override
	public String getSubsystem() {
		return subsystem;
	}

	@Override
	public void setSubsystem(String subsystem) {
		this.subsystem = subsystem;
	}

	@Override
	public void initSendable(SendableBuilder builder) {
		builder.setSmartDashboardType("Speed Controller");
		builder.setActuator(true);
		builder.setSafeState(this::disable);
		builder.addDoubleProperty("Value", controller::get, controller::set);
	}

	public double getFeedForward(double velocity) {
		return velocity/max_velocity;
	}

	public PIDController VelocityToPID(double velocity) {
		double F = getFeedForward(velocity); 
		return new PIDController(P, I, D, F, encoder, controller);
	}

	public PIDController CustomPID(double kP, double kI, double kD, double kF) {
		return new PIDController(kP, kI, kD, kF, encoder, controller);
	}
}
