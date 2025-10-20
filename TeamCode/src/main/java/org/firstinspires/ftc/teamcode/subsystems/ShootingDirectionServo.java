package org.firstinspires.ftc.teamcode.subsystems;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.ServoEx;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Servo Subsystem for the robot.
 *
 * This class controls a servo mechanism with position control.
 * It provides commands to set the servo to various positions (min, max, middle, custom).
 */
public class ShootingDirectionServo implements Subsystem {

    // Singleton instance of the ServoSubsystem (initially null)
    private static ShootingDirectionServo INSTANCE = null;

    // Servo instance - initialized with hardware map name
    private ServoEx servo;
    private static final String SERVO_NM = "sm_servo";

    // Telemetry for displaying servo status
    private Telemetry telemetry;

    // Private constructor to ensure only one instance exists
    private ShootingDirectionServo(Telemetry telemetry) {

        this.telemetry = telemetry;
        servo = new ServoEx(SERVO_NM);
    }

    /**
     * Gets or creates the singleton instance of the ServoSubsystem.
     * @param telemetry The telemetry object to use for displaying data
     * @return The singleton instance
     */
    public static ShootingDirectionServo getInstance(Telemetry telemetry) {
        if (INSTANCE == null) {
            INSTANCE = new ShootingDirectionServo(telemetry);
        }
        return INSTANCE;
    }

    /**
     * Gets the existing singleton instance (use only after getInstance(telemetry) has been called)
     * @return The singleton instance
     */
    public static ShootingDirectionServo getInstance() {
        if (INSTANCE == null) {
            throw new IllegalStateException("ServoSubsystem must be initialized with telemetry first!");
        }
        return INSTANCE;
    }

    // Servo positions
    private double minPos = 0.0;
    private double pos1 = 0.33;
    private double pos2 = 0.67;
    private double maxPos = 1.0;

    private static final double SERVO_POS_INCREMENT = 0.05;

    /**
     * Initialize the servo with a hardware map name
     * @param SERVO_NM The name of the servo in the hardware map
     */
    public void initialize(String SERVO_NM) {
        this.servo = new ServoEx(SERVO_NM);
//        this.servo.setdirection(ServoEx.Direction.FORWARD);
    }

    /**
     * Initialize the servo with custom positions
     * @param servoName The name of the servo in the hardware map
     * @param minPos Minimum position (0.0 to 1.0)
     * @param pos1 First intermediate position
     * @param pos2 Second intermediate position
     * @param maxPos Maximum position (0.0 to 1.0)
     */
    public void initialize(String servoName, double minPos, double pos1, double pos2, double maxPos) {
        this.servo = new ServoEx(servoName);
        this.minPos = minPos;
        this.pos1 = pos1;
        this.pos2 = pos2;
        this.maxPos = maxPos;
    }

    // Command to set servo to minimum position
    public Command setMinPos = new InstantCommand(() -> {
        if (servo != null) servo.setPosition(minPos);
    }).requires(this);

    // Command to set servo to first intermediate position
    public Command setPos1 = new InstantCommand(() -> {
        if (servo != null) servo.setPosition(pos1);
    }).requires(this);

    // Command to set servo to second intermediate position
    public Command setPos2 = new InstantCommand(() -> {
        if (servo != null) servo.setPosition(pos2);
    }).requires(this);

    // Command to set servo to maximum position
    public Command setMaxPos = new InstantCommand(() -> {
        if (servo != null) servo.setPosition(maxPos);
    }).requires(this);

    // Legacy commands for backward compatibility
//    public Command setMin = setMinPos;
//    public Command setMax = setMaxPos;
//    public Command setMiddle = setPos1;

    /**
     * Command to set servo to a custom position
     * @param increment The target position (0.0 to 1.0)
     * @return Command to set the position
     */
    public Command setPosition(double increment) {
        return new InstantCommand(() -> {
            if (servo != null) {
                double futurePos = servo.getPosition() + increment;
                double clampedPosition = Math.max(0.0, Math.min(1.0, futurePos));
                servo.setPosition(clampedPosition);
            }
        }).requires(this);
    }

    public Command upShootingServo = new InstantCommand(() -> {
        if (servo != null) {
            double clampedPosition = Math.min(1.0, servo.getPosition() + SERVO_POS_INCREMENT);
//                double clampedPosition = servo.getPosition() + SERVO_POS_INCREMENT;
            servo.setPosition(clampedPosition);
        }
    }).requires(this);
    public Command downShootingServo = new InstantCommand(() -> {
        if (servo != null) {
            double clampedPosition = Math.max(0.0, servo.getPosition() - SERVO_POS_INCREMENT);
//                double clampedPosition = servo.getPosition() - SERVO_POS_INCREMENT;
            servo.setPosition(clampedPosition);
        }
    }).requires(this);

    /**
     * Get the current servo position
     * @return Current position (0.0 to 1.0)
     */
    public double getPosition() {
        return servo != null ? servo.getPosition() : 0.0;
    }

    /**
     * Get the configured positions
     */
//    public double getMinPos() { return minPos; }
//    public double getPos1() { return pos1; }
//    public double getPos2() { return pos2; }
//    public double getMaxPos() { return maxPos; }

    /**
     * The periodic method is called repeatedly while the robot is running.
     * Displays telemetry data about the servo status.
     */
    @Override
    public void periodic() {
        telemetry.addData("<=====Servo Subsystem=====>", "");
        if (servo != null) {
            double currentPos = servo.getPosition();
            String positionName = getCurrentPositionName(currentPos);
            telemetry.addData("Current Position", positionName);
            telemetry.addData("Position Value", "%.3f", currentPos);
        } else {
            telemetry.addData("Servo Status", "Not Initialized");
        }
        //telemetry.update();
    }

    /**
     * Helper method to determine which position the servo is currently at
     * @param currentPos The current servo position
     * @return The name of the position (min, pos1, pos2, max, or "Custom")
     */
    private String getCurrentPositionName(double currentPos) {
        double tolerance = 0.01; // Tolerance for position comparison

        if (Math.abs(currentPos - minPos) < tolerance) {
            return "Min";
        } else if (Math.abs(currentPos - pos1) < tolerance) {
            return "Pos1";
        } else if (Math.abs(currentPos - pos2) < tolerance) {
            return "Pos2";
        } else if (Math.abs(currentPos - maxPos) < tolerance) {
            return "Max";
        } else {
            return "Custom";
        }
    }
}