package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="DriveCode", group="TeleOp")
public class DriveCode extends LinearOpMode {

    // Declare drive motors, other motors, and servos
    private DcMotor frontLeftMotor;
    private DcMotor backLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backRightMotor;
    private DcMotor sweepMotor;
    private DcMotor linearMotor;
    private DcMotor armMotor;
    private Servo servo;      // For other functions (e.g., grabbing)
    private Servo armServo;   // New servo whose position we track


    // Linear motor control variable
    private int linearTargetPosition = 5;  // Starting target

    // Arm motor control variables
    private static final int ARM_HALF_POSITION = 70; // Safe half-up position for the arm
    private int armTargetPosition = ARM_HALF_POSITION;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize hardware mappings
        frontLeftMotor  = hardwareMap.dcMotor.get("frontLeftMotor");
        backLeftMotor   = hardwareMap.dcMotor.get("backLeftMotor");
        frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        backRightMotor  = hardwareMap.dcMotor.get("backRightMotor");
        sweepMotor      = hardwareMap.dcMotor.get("sweepMotor");
        linearMotor     = hardwareMap.dcMotor.get("linearMotor");
        armMotor        = hardwareMap.dcMotor.get("armMotor");
        servo           = hardwareMap.servo.get("servo");
        armServo        = hardwareMap.servo.get("armServo");

        // Reverse the right-side drive motors if needed
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        // --- Initialize the Linear Motor ---
        linearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Initialize the target to the current encoder reading
        linearTargetPosition = linearMotor.getCurrentPosition();
        linearMotor.setTargetPosition(linearTargetPosition);
        linearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearMotor.setPower(1.0);

        // --- Initialize the Arm Motor ---
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armTargetPosition = ARM_HALF_POSITION;
        armMotor.setTargetPosition(armTargetPosition);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(0.3);

        // --- Initialize Servos ---
        servo.setPosition(0.42);

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            // === Driving Controls (Gamepad1 for drive train) ===
            double drive  = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double rotate = -gamepad1.right_stick_x;

            double frontLeftPower  = drive + strafe + rotate;
            double backLeftPower   = drive - strafe + rotate;
            double frontRightPower = drive - strafe - rotate;
            double backRightPower  = drive + strafe - rotate;

            // Normalize drive motor powers if any exceeds 1.0
            double maxPower = Math.max(Math.max(Math.abs(frontLeftPower), Math.abs(backLeftPower)),
                    Math.max(Math.abs(frontRightPower), Math.abs(backRightPower)));
            if(maxPower > 1.0){
                frontLeftPower  /= maxPower;
                backLeftPower   /= maxPower;
                frontRightPower /= maxPower;
                backRightPower  /= maxPower;
            }
            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            // === Linear Motor Control (Gamepad2) ===

            // Manual control with right joystick:
            if (Math.abs(gamepad2.right_stick_y) > 0.1) {
                // Invert joystick value so that pushing up increases the target position
                int increment = (int)(-gamepad2.right_stick_y * 30); // Adjust multiplier as needed for speed
                linearTargetPosition += increment;
                linearMotor.setTargetPosition(linearTargetPosition);
                linearMotor.setPower(1.0);
            }

            // Preset positions using D-pad
            else if (gamepad2.dpad_up) {
                linearTargetPosition = 13400;
                linearMotor.setTargetPosition(linearTargetPosition);
                linearMotor.setPower(1.0);
            }
            else if (gamepad2.dpad_down) {
                linearTargetPosition = 5;
                linearMotor.setTargetPosition(linearTargetPosition);
                linearMotor.setPower(1.0);
            }
            else {
                // When no input, continue holding the last target position.
                linearMotor.setTargetPosition(linearTargetPosition);
                linearMotor.setPower(1.0);
            }

            // === Arm Motor Control (Gamepad1) ===
            if (gamepad1.dpad_up) {
                // Set arm to the half-held up position
                armTargetPosition = ARM_HALF_POSITION;
                armMotor.setTargetPosition(armTargetPosition);
                armMotor.setPower(.5);
            }
            else if (gamepad1.dpad_right) {
                // Manually raise the arm while held
                armTargetPosition += 1;  // Adjust increment as needed
                armMotor.setTargetPosition(armTargetPosition);
                armMotor.setPower(.5);
            }
            else if (gamepad1.dpad_left) {
                // Manually lower the arm while held
                armTargetPosition -= 1;  // Adjust increment as needed
                armMotor.setTargetPosition(armTargetPosition);
                armMotor.setPower(.5);
            }
            else {
                // Hold the current arm position
                armMotor.setTargetPosition(armTargetPosition);
                armMotor.setPower(1.0);
            }

            // === Sweep Motor Control (Gamepad2 triggers) ===
            if (gamepad2.right_trigger > 0) {
                sweepMotor.setPower(0.4);
            } else if (gamepad2.left_trigger > 0) {
                sweepMotor.setPower(-0.4);
            } else {
                sweepMotor.setPower(0);
            }
// --- Arm Servo Control (Gamepad1 X and Y buttons) ---
// Adjust these values as needed. The increment value here (0.01) is chosen
// so that, with a typical loop rate, the servo moves at roughly 0.3 units per second.


            if (gamepad1.x) {
                // Target for closed position is 0.
                double targetPos = .2;
                armServo.setPosition(targetPos);

            }
            if (gamepad1.y) {
                // Target for open position is 3.
                double targetPos = .0;
                armServo.setPosition(targetPos);

            }

            // === Servo Control (Gamepad2 buttons) ===
            if (gamepad2.x) {
                servo.setPosition(0.42);
            } else if (gamepad2.y) {
                servo.setPosition(0.3);
            } else if (gamepad2.b) {
                servo.setPosition(1.0);
                sleep(500);
                servo.setPosition(0.42);
            }
            if (gamepad2.left_bumper) {
                AutonomousMode.runAutonomous(this, frontLeftMotor, backLeftMotor, frontRightMotor,
                        backRightMotor, sweepMotor, linearMotor, armMotor, servo);
            }
            // === Telemetry ===
            telemetry.addData("Servo Position", servo.getPosition());
            telemetry.addData("ArmServo Position", armServo.getPosition());
            telemetry.addData("Linear Motor Target", linearTargetPosition);
            telemetry.addData("Linear Motor Current", linearMotor.getCurrentPosition());
            telemetry.addData("Arm Motor Target", armTargetPosition);
            telemetry.addData("Arm Motor Current", armMotor.getCurrentPosition());
            telemetry.addData("Arm Servo Position", armServo.getPosition());
            telemetry.addData("Initial Arm Servo Position", armServo.getPosition());
            telemetry.update();
        }
    }
}