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

@TeleOp(name="Opmode2")
public class OpMode2 extends LinearOpMode {

    public Robot2 robot = new Robot2();
    public Gyro gyro;

    DcMotor leftDrive, rightDrive, middleDrive, armMotor;
    Servo leftClawServo, rightClawServo;

    int targetPosition;

    int initArmPosition;

    double leftPower, rightPower, middlePower, armPower, servoPower;

    private ElapsedTime runtime = new ElapsedTime();

    double raiseStart = 0;

    int lastPosition;

    double initialAngle;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        gyro = new Gyro(robot);
        gyro.resetAngle();
        initialAngle = robot.imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;

        targetPosition = 0;
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize motors
        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        middleDrive = hardwareMap.get(DcMotor.class, "middle_drive");
        armMotor = hardwareMap.get(DcMotor.class, "arm_motor");

        // resetEncoders();

        // Initialize servos
        leftClawServo = hardwareMap.get(Servo.class, "left_claw_servo");
        rightClawServo = hardwareMap.get(Servo.class, "right_claw_servo");

        leftClawServo.setDirection(Servo.Direction.FORWARD);

        //Reset Left/Right finger positions
        leftClawServo.setPosition(0.5);
        rightClawServo.setPosition(0.5);



        // Set modes for arm
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        armMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        // Set direction to reverse for left drive motor
        leftDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set zero power behavior for drive motors
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        middleDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

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
            telemetry.addData("Run Time:", runtime.toString());


            // Drivetrain controls
            // Left motor - left stick
            // Right motor - right stick
            // Middle motor - left/right triggers
            leftPower  = -gamepad1.left_stick_y;
            rightPower = gamepad1.right_stick_y;
            if (gamepad1.left_trigger != 0) {
                middlePower = -gamepad1.left_trigger;
            } else if (gamepad1.right_trigger != 0) {
                middlePower = gamepad1.right_trigger;
            } else {
                middlePower = 0;
            }

            int armPosition = armMotor.getCurrentPosition();

            if (gamepad2.a || gamepad2.y || gamepad2.x) {
                if (gamepad2.y) {
                    // Raise arm
                    armPower = 1;
                } else if (gamepad2.a) {
                    // Lower arm down
                    armPower = -0.6;
                } else if (gamepad2.x) {
                    // Hold arm in place
                    armPower = (Math.abs(armPosition - initArmPosition) < 250) ? 0.6 : 0.5;
                }
            } else {
                // Check if arm hasn't moved, set to zero else slowly release
                if (armPosition == lastPosition) {
                    armPower = 0;
                } else {
                    armPower = 0.2;
                }
            }

            armMotor.setPower(armPower);

            lastPosition = armPosition;

            if (gamepad2.right_trigger > 0) {
                //openClaw();
                rightClawServo.setPosition(rightClawServo.getPosition() + 0.01);
                leftClawServo.setPosition(leftClawServo.getPosition() - 0.01);
            }
            if (gamepad2.left_trigger > 0) {
                //resetClaw();
                rightClawServo.setPosition(rightClawServo.getPosition() - 0.01);
                leftClawServo.setPosition(leftClawServo.getPosition() + 0.01);
            }


            // Send calculated power to wheels
            setDrivePower();

            telemetry.addData("Left drive position", leftDrive.getCurrentPosition());
            telemetry.addData("Right drive position", rightDrive.getCurrentPosition());
            telemetry.addData("Arm Motor position", armMotor.getCurrentPosition());
            telemetry.addData("Left servo position", leftClawServo.getPosition());
            telemetry.addData("Right servo position", rightClawServo.getPosition());
            telemetry.update();
        }
    }

    public void resetEncoders() {
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        middleDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void setDrivePower() {
        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);
        middleDrive.setPower(middlePower);
    }

    public void resetClaw() {
        leftClawServo.setPosition(0.80);
        rightClawServo.setPosition(0.85);
    }

    public void openClaw() {
        leftClawServo.setPosition(1);
        rightClawServo.setPosition(1);
    }
}