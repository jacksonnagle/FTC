package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class DriveCode extends LinearOpMode {

    // Declare motors and servo
    private DcMotor frontLeftMotor;
    private DcMotor backLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backRightMotor;
    private DcMotor sweepMotor;
    private DcMotor linearMotor;
    private DcMotor armMotor;
    private Servo servo;

    // Arm motor control variables
    private static final int ARM_HALF_POSITION = 100; // Safe half-up position
    private int armTargetPosition = ARM_HALF_POSITION;
    private boolean armHalfMode = true;
    private boolean leftDpadPressed = false; // For toggling arm mode (edge detection)

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize motors and servo from the hardware map
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        backRightMotor = hardwareMap.dcMotor.get("backRightMotor");
        sweepMotor = hardwareMap.dcMotor.get("sweepMotor");
        linearMotor = hardwareMap.dcMotor.get("linearMotor");
        armMotor = hardwareMap.dcMotor.get("armMotor");
        servo = hardwareMap.servo.get("servo");

        // Reverse right side motors if necessary
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // --- Initialize Linear Motor ---
        // Reset encoder, set initial target, and enable RUN_TO_POSITION mode so it can hold its position.
        linearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearMotor.setTargetPosition(0);
        linearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearMotor.setPower(1.0);

        // --- Initialize Arm Motor ---
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armTargetPosition = ARM_HALF_POSITION;
        armMotor.setTargetPosition(armTargetPosition);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(0.3);

        // Set the servo to its neutral position.
        servo.setPosition(0.42);

        waitForStart();
        if (isStopRequested()) return;

        // Variables for edge detection on d-pad for linear motor control
        boolean prevDpadUp = false;
        boolean prevDpadDown = false;

        while (opModeIsActive()) {
            // === Driving Controls (gamepad1) ===
            double drive = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double rotate = -gamepad1.right_stick_x;

            double frontLeftPower = drive + strafe + rotate;
            double backLeftPower = drive - strafe + rotate;
            double frontRightPower = drive - strafe - rotate;
            double backRightPower = drive + strafe - rotate;

            // Normalize the drive motor powers so that none exceeds 1.0
            double maxPower = Math.max(Math.max(Math.abs(frontLeftPower), Math.abs(backLeftPower)),
                    Math.max(Math.abs(frontRightPower), Math.abs(backRightPower)));
            if (maxPower > 1.0) {
                frontLeftPower /= maxPower;
                backLeftPower /= maxPower;
                frontRightPower /= maxPower;
                backRightPower /= maxPower;
            }
            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            // === Linear Motor Control (gamepad1 d-pad preset positions with edge detection) ===
            boolean currentDpadUp = gamepad1.dpad_up;
            boolean currentDpadDown = gamepad1.dpad_down;

            if (currentDpadUp && !prevDpadUp) {
                // On the rising edge of dpad_up, increase the target by 100 counts
                int newTargetPosition = linearMotor.getTargetPosition() + 100;
                linearMotor.setTargetPosition(newTargetPosition);
                linearMotor.setPower(1.0);
            } else if (currentDpadDown && !prevDpadDown) {
                // On the rising edge of dpad_down, decrease the target by 100 counts
                int newTargetPosition = linearMotor.getTargetPosition() - 100;
                linearMotor.setTargetPosition(newTargetPosition);
                linearMotor.setPower(1.0);
            } else {
                // When no new d-pad input, continue holding the current target
                linearMotor.setTargetPosition(linearMotor.getTargetPosition());
                linearMotor.setPower(1.0);
            }
            // Update previous d-pad states for edge detection next loop iteration
            prevDpadUp = currentDpadUp;
            prevDpadDown = currentDpadDown;

            // === Arm Motor Control (gamepad2) ===
            // Toggle between "half-hold" and manual mode using the left D-pad button on gamepad2.
            if (gamepad2.dpad_left && !leftDpadPressed) {
                armHalfMode = !armHalfMode;
                if (armHalfMode) {
                    armTargetPosition = ARM_HALF_POSITION;
                }
            }
            leftDpadPressed = gamepad2.dpad_left;

            // In manual mode, adjust the arm target slowly using D-pad up/down.
            if (!armHalfMode) {
                if (gamepad2.dpad_up) {
                    armTargetPosition += 10;
                }
                if (gamepad2.dpad_down) {
                    armTargetPosition -= 10;
                }
            }
            armMotor.setTargetPosition(armTargetPosition);
            armMotor.setPower(0.3);

            // === Sweep Motor Control (gamepad2 triggers) ===
            if (gamepad2.right_trigger > 0) {
                sweepMotor.setPower(0.4);
            } else if (gamepad2.left_trigger > 0) {
                sweepMotor.setPower(-0.4);
            } else {
                sweepMotor.setPower(0);
            }

            // === Servo Control (gamepad2 buttons) ===
            if (gamepad2.x) {
                servo.setPosition(0.42);
            } else if (gamepad2.y) {
                servo.setPosition(0.3);
            } else if (gamepad2.b) {
                servo.setPosition(1.0);
                sleep(500);
                servo.setPosition(0.42);
            }

            // === Autonomous Mode Trigger (if desired) ===
            if (gamepad2.left_bumper) {
                AutonomousMode.runAutonomous(this, frontLeftMotor, backLeftMotor, frontRightMotor,
                        backRightMotor, sweepMotor, linearMotor, armMotor, servo);
            }

            // === Telemetry ===
            telemetry.addData("Servo Position", servo.getPosition());
            telemetry.addData("Linear Motor Target", linearMotor.getTargetPosition());
            telemetry.addData("Linear Motor Current", linearMotor.getCurrentPosition());
            telemetry.addData("Arm Motor Target", armTargetPosition);
            telemetry.addData("Arm Motor Current", armMotor.getCurrentPosition());
            telemetry.addData("Arm Mode", armHalfMode ? "Half Mode" : "Manual Mode");
            telemetry.update();
        }
    }
}