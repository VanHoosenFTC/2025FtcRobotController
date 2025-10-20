/* This is updated code as of 10192025 */
package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.teamcode.ChassisConstants.LEFT_FRONT_MOTOR_NAME;
import static org.firstinspires.ftc.teamcode.ChassisConstants.LEFT_REAR_MOTOR_NAME;
import static org.firstinspires.ftc.teamcode.ChassisConstants.RIGHT_FRONT_MOTOR_NAME;
import static org.firstinspires.ftc.teamcode.ChassisConstants.RIGHT_REAR_MOTOR_NAME;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.ShootingDirectionServo;
import org.firstinspires.ftc.teamcode.subsystems.ShootingSystem;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.driving.MecanumDriverControlled;
import dev.nextftc.hardware.impl.MotorEx;

@TeleOp(name = "Robot Centric TeleOp")
public class RobotCentricTeleOp extends NextFTCOpMode {

    //private Telemetry telemetry;
    private ShootingSystem shootingSystem;
    private ShootingDirectionServo shootingDirectionServo;
    private Intake intakeSystem;

    public RobotCentricTeleOp() {
        //telemetry = super.telemetry;
        shootingSystem = ShootingSystem.getInstance(telemetry);
        intakeSystem = Intake.getInstance(telemetry);
        shootingDirectionServo = ShootingDirectionServo.getInstance(telemetry);
        addComponents(
                new SubsystemComponent(shootingSystem),
                new SubsystemComponent(intakeSystem),
                new SubsystemComponent(shootingDirectionServo),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }



    // change the names and directions to suit your robot
    private final MotorEx frontLeftMotor = new MotorEx(LEFT_FRONT_MOTOR_NAME);
    private final MotorEx frontRightMotor = new MotorEx(RIGHT_FRONT_MOTOR_NAME).reversed();
    private final MotorEx backLeftMotor = new MotorEx(LEFT_REAR_MOTOR_NAME);
    private final MotorEx backRightMotor = new MotorEx(RIGHT_REAR_MOTOR_NAME).reversed();

    @Override
    public void onStartButtonPressed() {
        Command driverControlled = new MecanumDriverControlled(
                frontLeftMotor,
                frontRightMotor,
                backLeftMotor,
                backRightMotor,
                Gamepads.gamepad1().leftStickY(),
                Gamepads.gamepad1().leftStickX(),
                Gamepads.gamepad1().rightStickX()
        );
        driverControlled.schedule();

        // Shooting System Controls on Gamepad 2
        Gamepads.gamepad2().x().whenBecomesTrue(shootingSystem.startStop);
        Gamepads.gamepad2().y().whenBecomesTrue(shootingSystem.toggleShootingPower);
        Gamepads.gamepad2().leftBumper().whenBecomesTrue(shootingSystem.decreaseShootingPower);
        Gamepads.gamepad2().rightBumper().whenBecomesTrue(shootingSystem.increaseShootingPower);

        // Intake System Controls on Gamepad 2
        Gamepads.gamepad2().b().whenBecomesTrue(intakeSystem.reverse);
        Gamepads.gamepad2().a().whenBecomesTrue(intakeSystem.startStop);

        // Shooting Direction Servo Controls on Gamepad 2
        Gamepads.gamepad2().dpadUp().whenBecomesTrue(shootingDirectionServo.downShootingServo);
        Gamepads.gamepad2().dpadDown().whenBecomesTrue(shootingDirectionServo.upShootingServo);
    }

}