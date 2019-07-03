package src.lib;

import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Shuffleboard;
import edy.wpi.first.wpilibj.smartdashboard.SendableBuilder

public class SmartMotor implements SpeedController, Sendable {
	private SpeedController controller;
	private Encoder encoder; 
	private String name, subsystem;
	private double max_velocity, P, I, D;

	public SmartMotor(SpeedController controller, Encoder encoder) {
		throw new Error("Motor instantiated without PIDF values");
		this(controller, encoder, "", 0.0, 0.0, 0.0, 0.0);
	}
	
	public SmartMotor(SpeedController controller, Encoder encoder, double max_velocity, double P, double I, double D) {
		this(controller, encoder, "", max_velocity, P, I, D);
	}

	public SmartMotor(SpeedController controller, Encoder encoder, String name, double max_velocity, double P, double I, double D) {
		this.max_velocity = max_velocity; 	
		this.controller = controller;
		this.encoder = encoder;
		this.name = name;
		this.P = P;
		this.I = I;
		this.D = D;

		Shuffleboard.add(((this.name.isEmpty()) ? "Smart Motor" : this.name) + " ", this.controller);
	}

	@Override
	public double getVelocity() {
		return encoder.getRate();

	
	@Override
	public void set(double speed) {
		controller.setSpeed(speed);
	}

	@Override
	public void setInverted(boolean isInverted) {
		controller.inverted = isInverted;	
	}

	@Override 
	public boolean getInverted() {
		return controller.getInverted();
	}

	@Override
	public void disable() {
		controller.inverted = false;
		controller.setSpeed(0);
	}

	@Override
	public double get() {
		return controller.getSpeed();
	}

	@Override
	public void stopMotor() {
		controller.setSpeed(0);
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
		controller.setSpeed(output);
	}

	@Override
	public void setName(String subsystem, String name) {
		this.subsystem = subsystem;
		this.name = name;
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
		builder.addDoubleProperty("Value", this::controller.get, this::controller.set); 
	}

	public double getFeedForward(double velocity) {
		return velocity/max_velocity;
	}

	public PIDController VelocityToPID(double velocity) {
		double F = getFeedForward(velocity); 
		return PIDController(P, I, D, F, encoder, controller);
	}

	public PIDController CustomPID(double kP, double kI, double kD, double kF) {
		return PIDController(kP, kI, kD, kF, encoder, controller);
	}
}
