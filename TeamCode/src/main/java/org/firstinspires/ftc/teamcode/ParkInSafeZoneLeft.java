package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by WTR on 11/8/2017.
 */
@Autonomous(name="ParkInSafeZoneLeft", group="WTR")
//@Disabled
public class ParkInSafeZoneLeft extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime stateTime = new ElapsedTime();

    private enum state {
        STATE_MOVE_FORWARD,
        STATE_TURN_NINETY,
        STATE_MOVE_MORE,

    }

    private state currentState = null;

    //Motors
    public DcMotor leftMotor = null;
    public DcMotor rightMotor = null;

    @Override
    public void init() {
        leftMotor = hardwareMap.dcMotor.get("leftmotor");
        rightMotor = hardwareMap.dcMotor.get("rightmotor");

        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        rightMotor.setDirection(DcMotor.Direction.FORWARD);

    }

    @Override
    public void init_loop() {
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    @Override
    public void start(){
        runtime.reset();
        newState(state.STATE_MOVE_FORWARD);
    }

    @Override
    public void loop() {
        switch (currentState) {
            case STATE_MOVE_FORWARD:
                setPos(36,1);
                newState(state.STATE_TURN_NINETY);
                break;

            case STATE_TURN_NINETY:
                if (stateTime.time() <=1.5) {
                    leftMotor.setPower(-1);
                    rightMotor.setPower(1);
                } else {
                    newState(state.STATE_MOVE_MORE);
                }
                break;

            case STATE_MOVE_MORE:
                setPos(12,1);
                break;
        }

    }


    public void stop(){
    }

    private void newState(state newState) {
        stateTime.reset();
        currentState = newState;
    }

    public void setPos(double inches, double goes){
        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        int ticks = (int) (inches * (118.8356));
        //adjust inch to tick conversion to new robot

        int currentLeft = leftMotor.getCurrentPosition();
        int currentRight = rightMotor.getCurrentPosition();

        leftMotor.setTargetPosition(ticks + currentLeft);
        rightMotor.setTargetPosition(ticks + currentRight);
        leftMotor.setPower(goes);
        rightMotor.setPower(goes);
    }

    public double currentPos = 0;
}
