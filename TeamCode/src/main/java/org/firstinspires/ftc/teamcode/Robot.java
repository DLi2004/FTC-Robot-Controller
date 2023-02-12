package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.EventLoopManager;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class Robot {
    HardwareMap hwMap = null;
    // Declare OpMode members.
    public ElapsedTime runtime = new ElapsedTime();

    public DcMotor leftDrive = null;
    public DcMotor rightDrive = null;
    public DcMotor middleDrive = null;
    public DcMotor armMotor = null;
    public Servo leftClawServo = null;
    public Servo rightClawServo = null;

    public int initArmPosition = 0;

    BHI260IMU imu;
    IMU.Parameters IMUParameters;

    public void startRobot(HardwareMap ahwmap){
        hwMap = ahwmap;

        //  Get all drive motors
        leftDrive = hwMap.get(DcMotor.class, "left_drive");
        rightDrive = hwMap.get(DcMotor.class, "right_drive");
        middleDrive = hwMap.get(DcMotor.class, "middle_drive");
        armMotor = hwMap.get(DcMotor.class, "arm_motor");

        // Left side drive motor is reverse
        leftDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        // Right side drive motor and middle motor is forward
        rightDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        middleDrive.setDirection(DcMotorSimple.Direction.FORWARD);

        //Set all drive motors to zero power brake
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        middleDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Set all drive motors to zero power
        setAllDrivePower(0);
        //Set all drive motors to run without encoders
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        middleDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Set arm motors to work in tandem
        armMotor.setDirection(DcMotor.Direction.FORWARD);

        //Set arm run without encoder
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Set arm motors brake mode
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Set initial arm motor position
        initArmPosition = armMotor.getCurrentPosition();

        //Get claw servos
        leftClawServo = hwMap.get(Servo.class, "left_claw_servo");
        rightClawServo = hwMap.get(Servo.class, "right_claw_servo");

        //Set claw left finger to work in tandem w/ right finger
        leftClawServo.setDirection(Servo.Direction.REVERSE);

        //Reset Left/Right finger positions
        leftClawServo.setPosition(0);
        rightClawServo.setPosition(0);

        IMUParameters = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));

        // Get IMU
        imu = hwMap.get(BHI260IMU.class, "imu");
        imu.initialize(IMUParameters);
    }

    public void setAllDrivePower(double p){
        setDrivePower(p,p,p,p);
    }

    public void setDrivePower(double lF, double rF, double lB, double rB){
        leftDrive.setPower(lF);
        rightDrive.setPower(lB);
        middleDrive.setPower(rF);
        armMotor.setPower(rB);
    }
}