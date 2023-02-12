package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import java.lang.reflect.Array;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.Arrays;
import java.lang.Math;

@TeleOp(name="OpMode")
public class OpMode extends LinearOpMode {

    DcMotor leftDrive, rightDrive, middleDrive, armMotor;
    Servo leftClawServo, rightClawServo;

    int targetPosition;

    int initArmPosition;

    double leftPower, rightPower, middlePower, armPower, servoPower;

    private ElapsedTime runtime = new ElapsedTime();

    private ArmPIDController armPIDController;

    int lastPosition;

    @Override
    public void runOpMode() {
        targetPosition = 0;
        armPIDController = new ArmPIDController(targetPosition, 0.03,0.00001,0.01, runtime);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize motors
        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        middleDrive = hardwareMap.get(DcMotor.class, "middle_drive");
        armMotor = hardwareMap.get(DcMotor.class, "arm_motor");

        // Initialize servos
        // leftClawServo = hardwareMap.get(Servo.class, "left_claw_servo");
        // rightClawServo = hardwareMap.get(Servo.class, "right_class_servo");

        // Set modes for arm
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        // Set direction to reverse for left drive motor
        leftDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set zero power behavior for drive motors
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        middleDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize arm motor info
        initArmPosition = armMotor.getCurrentPosition();
        lastPosition = initArmPosition;
        targetPosition = initArmPosition;
        armMotor.setTargetPosition(initArmPosition);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // Show the elapsed game time
            telemetry.addData("Status", "Run Time: " + runtime.toString());


            // Drivetrain controls
            // Left motor - left stick
            // Right motor - right stick
            // Middle motor - left/right triggers
            leftPower  = -gamepad1.left_stick_y;
            rightPower = gamepad1.right_stick_y;
            if (gamepad1.left_trigger != 0) {
                middlePower = gamepad1.left_trigger;
            } else if (gamepad1.right_trigger != 0) {
                middlePower = -gamepad1.right_trigger;
            } else {
                middlePower = 0;
            }

            int armPosition = armMotor.getCurrentPosition();

            if (gamepad2.a || gamepad2.y || gamepad2.x) {
                if (gamepad2.y && armPosition <= initArmPosition && armPosition >= initArmPosition - 500) {
                    armPower = 0.5;
                } else if (gamepad2.a && armPosition >= initArmPosition - 700 && armPosition <= initArmPosition - 500) {
                    //updateTargetPosition(targetPosition - 10);
                    armPower = -0.5;
                } else if (gamepad2.x) {
                    armPower = (Math.abs(armPosition - initArmPosition) < 250) ? 0.3 : 0.2;
                }
            } else {
                // Check if arm hasn't moved, set to zero else slowly release
                armPower = (armPosition == lastPosition) ? 0 : 0.05;
            }

            armMotor.setPower(armPower);

            lastPosition = armPosition;

            // if (gamepad2.right_trigger != 0) {
            //     openClaw();
            // } else if (gamepad2.left_trigger != 0) {
            //     gripClaw();
            // } else if (gamepad2.left_bumper) {
            //     resetClaw();
            // }


            // Send calculated power to wheels
            setDrivePower();

//            telemetry.addData("a pressed", gamepad1.a);
//            telemetry.addData("x pressed", gamepad1.x);
//            telemetry.addData("y pressed", gamepad1.y);
//            telemetry.addData("b pressed", gamepad1.b);
            telemetry.addData("target", targetPosition);
            telemetry.addData("init", initArmPosition);
            telemetry.addData("Arm Motor position", armMotor.getCurrentPosition());
            telemetry.addData("position<init", armPosition < initArmPosition);
            telemetry.addData("position>init-600", armPosition > initArmPosition - 600);
            telemetry.addData("Left drive position", leftDrive.getCurrentPosition());
            telemetry.addData("Right drive position", rightDrive.getCurrentPosition());
            telemetry.addData("Arm motor target", Integer.toString(targetPosition));
            telemetry.addData("Arm motor power", armPower);
            telemetry.update();
        }
    }

    public void setDrivePower() {
        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);
        middleDrive.setPower(middlePower);
    }

    public void updateTargetPosition(int newTargetPosition) {
        targetPosition = newTargetPosition;
        armMotor.setTargetPosition(targetPosition);
        armPIDController.updateTarget(newTargetPosition);
    }

     public void openClaw() {
         leftClawServo.setPosition(0.1);
         rightClawServo.setPosition(0.1);
     }

     public void resetClaw() {
         leftClawServo.setPosition(0);
         rightClawServo.setPosition(0);
     }

     public void gripClaw() {
         leftClawServo.setPosition(0.05);
         rightClawServo.setPosition(0.05);
     }
}