package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name = "RelicRecoveryTeleOp", group = "WTR")
//@Disabled

public class RelicRecoveryTeleOp extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime stateTime = new ElapsedTime();

    //Motors
    public DcMotor leftMotor = null;
    //public DcMotor leftMotorTwo = null;
    public DcMotor rightMotor = null;
    //public DcMotor rightMotorTwo = null;
    public DcMotor elevatorMotor = null;
    public DcMotor intakeMotorRight = null;
    public DcMotor intakeMotorLeft = null;


    public ServoController servoController = null;

    //300 servos
    public Servo dumpingServoLeft = null;
    public Servo dumpingServoRight = null;

    //180 servos
    public Servo leftDoorServo = null;
    public Servo rightDoorServo = null;

    @Override
    public void init() {
        //Motors
        leftMotor = hardwareMap.dcMotor.get("leftmotor");
        //leftMotorTwo = hardwareMap.dcMotor.get("leftmotortwo");
        rightMotor = hardwareMap.dcMotor.get("leftmotor");
        //rightMotorTwo = hardwareMap.dcMotor.get("rightmotortwo");
        elevatorMotor = hardwareMap.dcMotor.get("elevatormotor");
        intakeMotorRight = hardwareMap.dcMotor.get("inright");
        intakeMotorLeft = hardwareMap.dcMotor.get("inleft");

        servoController = hardwareMap.servoController.get("servocontroll");

        //300 servos
        dumpingServoLeft = hardwareMap.servo.get("dumpingservoone");
        dumpingServoRight = hardwareMap.servo.get("dumpingservotwo");

        //180 servos
        leftDoorServo = hardwareMap.servo.get("leftdoorservo");
        rightDoorServo = hardwareMap.servo.get("rightdoorservo");


        //Modes
        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        //leftMotorTwo.setDirection(DcMotor.Direction.REVERSE);
        rightMotor.setDirection(DcMotor.Direction.FORWARD);
        //rightMotorTwo.setDirection(DcMotor.Direction.FORWARD);
        intakeMotorLeft.setDirection(DcMotor.Direction.FORWARD);
        intakeMotorRight.setDirection(DcMotor.Direction.REVERSE);
        elevatorMotor.setDirection(DcMotor.Direction.FORWARD);
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start(){
        runtime.reset();
    }

    @Override
    public void loop(){

        //Drive Train
        if (Math.abs(gamepad1.left_stick_y) < .15) {
            leftMotor.setPower(0);
            //leftMotorTwo.setPower(0);
        } else {
            leftMotor.setPower(-gamepad1.left_stick_y);
            //leftMotorTwo.setPower(-gamepad1.left_stick_y);
        }
        if (Math.abs(gamepad1.right_stick_y) < .15) {
            rightMotor.setPower(0);
            //rightMotorTwo.setPower(0);
        } else {
            rightMotor.setPower(-gamepad1.right_stick_y);
            //rightMotorTwo.setPower(-gamepad1.right_stick_y);
        }

        //Intake
        if (gamepad1.right_bumper){
            intakeMotorRight.setPower(1);
            intakeMotorLeft.setPower(-1);
        }

        //Outtake
        if (gamepad1.left_bumper){
            intakeMotorRight.setPower(-1);
            intakeMotorLeft.setPower(1);
        }

        //open
        if (gamepad2.a) {
            setLeftDoorServo(-1);
            setRightDoorServo(1);
        }

        //close
        if (gamepad2.b) {
            setLeftDoorServo(0);
            setRightDoorServo(0);
        }

        //FLIPPP
        if (gamepad2.y) {
            setLeftDumpServo(1);
            setRightDumpServo(1);
        } if (!gamepad2.y){
            setLeftDumpServo(0);
            setRightDumpServo(0);
        }

        //upadoodles
        if (gamepad2.right_bumper) {
            elevatorMotor.setPower(1);
        }

        //downadoodles
        if (gamepad2.left_bumper) {
            elevatorMotor.setPower(-1);
        }



    }

    @Override
    public void stop(){
    }

    public double currentPos = 0;

    //Set Servo functions

    /*LeftDoor*/
    public void setLeftDoorServo(double pos) {
        currentPos = pos;
        leftDoorServo.setPosition(pos);
    }

    /*RightDoor*/
    public void setRightDoorServo(double pos) {
        currentPos = pos;
        rightDoorServo.setPosition(pos);
    }

    /*DumpLeft*/
    public void setLeftDumpServo(double pos) {
        currentPos = pos;
        dumpingServoLeft.setPosition(pos);
    }

    /*DumpRight*/
    public void setRightDumpServo(double pos) {
        currentPos = pos;
        dumpingServoRight.setPosition(pos);
    }




}
