package org.firstinspires.ftc.teamcode.subsystems;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Intake Subsystem for the robot.
 *
 * This class controls the intake mechanism using a single motor with simple power control.
 * It provides commands to start (intake), stop, and reverse (outtake) the intake.
 */
public class Intake implements Subsystem {

    // Singleton instance of the Intake subsystem (initially null)
    private static Intake INSTANCE = null;

    // Telemetry for displaying motor status
    private Telemetry telemetry;

    // Private constructor to ensure only one instance exists
    private Intake(Telemetry telemetry) {

        this.telemetry = telemetry;
    }

    /**
     * Gets or creates the singleton instance of the Intake subsystem.
     * @param telemetry The telemetry object to use for displaying data
     * @return The singleton instance
     */
    public static Intake getInstance(Telemetry telemetry) {
        if (INSTANCE == null) {
            INSTANCE = new Intake(telemetry);
        }
        return INSTANCE;
    }

    /**
     * Gets the existing singleton instance (use only after getInstance(telemetry) has been called)
     * @return The singleton instance
     */
    public static Intake getInstance() {
        if (INSTANCE == null) {
            throw new IllegalStateException("Intake must be initialized with telemetry first!");
        }
        return INSTANCE;
    }

    // Motor instance representing the intake motor
    private MotorEx motor = new MotorEx("intakeMotor");

    // Power levels for different intake states
    private static final double REVERSE_POWER = 1.0;   // Full power intake
    private static final double STOP_POWER = 0.0;      // Motor off
    private static final double INTAKE_POWER = -1.0;  // Full power reverse

    // Command to start the intake motor (intake game elements)
    public Command startStop = new InstantCommand(() -> {
        if (motor.getPower() <= 0.10 && motor.getPower() >= 0.0) motor.setPower(INTAKE_POWER);
        else motor.setPower(STOP_POWER);
    }).requires(this);

    // Command to stop the intake motor
//    public Command stop = new InstantCommand(() -> motor.setPower(STOP_POWER)).requires(this);

    // Command to reverse the intake motor direction (outtake/eject)
    public Command reverse = new InstantCommand(() -> motor.setPower(REVERSE_POWER))
            .requires(this);

    /**
     * The periodic method is called repeatedly while the robot is running.
     * Displays telemetry data about the intake motor status.
     */
    @Override
    public void periodic() {
        telemetry.addData("<=====Intake Subsystem=====>","");
        telemetry.addData("Intake Power", "%.2f", motor.getPower());
        telemetry.addData("Intake Direction", motor.getDirection());
        telemetry.addData("Intake Velocity", "%.2f", motor.getVelocity());
        //telemetry.update();
    }
}