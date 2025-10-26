package org.firstinspires.ftc.teamcode.subsystems;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.ServoEx;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Servo Subsystem for the robot.
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
    private final double minPos = 1.0;
    private final double maxPos = 0.8;

    private static final double SERVO_POS_INCREMENT = 0.05;

    public Command upShootingServo = new InstantCommand(() -> {
        if (servo != null) {
            double clampedPosition = Math.min(minPos, servo.getPosition() + SERVO_POS_INCREMENT);
            servo.setPosition(clampedPosition);
        }
    }).requires(this);
    public Command downShootingServo = new InstantCommand(() -> {
        if (servo != null) {
            double clampedPosition = Math.max(maxPos, servo.getPosition() - SERVO_POS_INCREMENT);
            servo.setPosition(clampedPosition);
        }
    }).requires(this);

    /**
     * The periodic method is called repeatedly while the robot is running.
     * Displays telemetry data about the servo status.
     */
    @Override
    public void periodic() {
        telemetry.addData("<=====Shooting Direction Servo=====>", "");
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
        } else if (Math.abs(currentPos - maxPos) < tolerance) {
            return "Max";
        } else {
            return "Custom";
        }
    }
}