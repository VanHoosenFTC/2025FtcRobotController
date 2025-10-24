package org.firstinspires.ftc.teamcode.subsystems;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.CRServoEx;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Servo Subsystem for the robot with continuous rotation.
 *
 * This class controls a goBILDA dual-mode servo in continuous rotation mode.
 * The servo must be physically switched to CR mode on the hardware.
 */
public class BallLoadingServo implements Subsystem {

    // Singleton instance of the ServoSubsystem (initially null)
    private static BallLoadingServo INSTANCE = null;

    // Servo instance for continuous rotation
    private CRServoEx crServo;
    private static final String SERVO_NM = "ld_servo";

    // Current power tracking
    private double currentPower = 0.0;

    // Telemetry for displaying servo status
    private Telemetry telemetry;

    // Private constructor to ensure only one instance exists
    private BallLoadingServo(Telemetry telemetry) {
        this.telemetry = telemetry;
        crServo = new CRServoEx(SERVO_NM);
        crServo.setPower(0.0); // Initialize stopped
    }

    /**
     * Gets or creates the singleton instance of the ServoSubsystem.
     * @param telemetry The telemetry object to use for displaying data
     * @return The singleton instance
     */
    public static BallLoadingServo getInstance(Telemetry telemetry) {
        if (INSTANCE == null) {
            INSTANCE = new BallLoadingServo(telemetry);
        }
        return INSTANCE;
    }

    /**
     * Gets the existing singleton instance (use only after getInstance(telemetry) has been called)
     * @return The singleton instance
     */
    public static BallLoadingServo getInstance() {
        if (INSTANCE == null) {
            throw new IllegalStateException("ServoSubsystem must be initialized with telemetry first!");
        }
        return INSTANCE;
    }

    /**
     * Set the continuous rotation power.
     * @param power Power value from -1.0 (full reverse) to 1.0 (full forward), 0.0 = stop
     * @return Command to set the power
     */
    public Command setContinuousPower(double power) {
        return new InstantCommand(() -> {
            if (crServo != null) {
                currentPower = Math.max(-1.0, Math.min(1.0, power)); // Clamp between -1 and 1
                crServo.setPower(currentPower);
            }
        });
    }

    /**
     * Run servo forward at specified power.
     * @param power Power from 0.0 to 1.0 (default: 1.0 for full speed)
     * @return Command to run forward
     */
    public Command runForward(double power) {
        return setContinuousPower(Math.abs(power));
    }

    /**
     * Run servo forward at full speed.
     * @return Command to run forward
     */
    public Command runForward() {
        return runForward(1.0);
    }

    /**
     * Run servo backward at specified power.
     * @param power Power from 0.0 to 1.0 (default: 1.0 for full speed)
     * @return Command to run backward
     */
    public Command runBackward(double power) {
        return setContinuousPower(-Math.abs(power));
    }

    /**
     * Run servo backward at full speed.
     * @return Command to run backward
     */
    public Command runBackward() {
        return runBackward(1.0);
    }

    /**
     * Stop the continuous rotation servo.
     * @return Command to stop
     */
    public Command stopContinuous() {
        return setContinuousPower(0.0);
    }

    /**
     * The periodic method is called repeatedly while the robot is running.
     * Displays telemetry data about the servo status.
     */
    @Override
    public void periodic() {
        telemetry.addData("<=====Servo Subsystem=====>", "");
        telemetry.addData("CR Power", "%.3f", currentPower);
        String status = currentPower > 0 ? "Forward" :
                currentPower < 0 ? "Backward" : "Stopped";
        telemetry.addData("CR Status", status);
    }
}