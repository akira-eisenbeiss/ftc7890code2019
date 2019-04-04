package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.StateMachine.State;

import java.util.ArrayList;

public class rangeCalibrationState implements State{
    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftBack;
    DcMotor rightBack;
    int Power;
    State NextState;
    ModernRoboticsI2cRangeSensor sideSensor1, sideSensor2;
    double dist;

    public rangeCalibrationState( ArrayList<DcMotor> motor,  ArrayList<ModernRoboticsI2cRangeSensor> mrrs){
        leftFront = motor.get(0);
        rightFront = motor.get(1);
        leftBack = motor.get(2);
        rightBack = motor.get(3);
        sideSensor1 = mrrs.get(0);
        sideSensor2 = mrrs.get(1);


    }
    public void setNextState(State state) {
        NextState  = state;

    }
    public void start(){

    }

    public State update(){


            if(sideSensor1.getDistance(DistanceUnit.INCH) > sideSensor2.getDistance(DistanceUnit.INCH)){
                move("CW", 0.3);
                return this;
            } else if (sideSensor1.getDistance(DistanceUnit.INCH) < sideSensor2.getDistance(DistanceUnit.INCH)){
                move("CCW", 0.3);
                return this;
            } else {

                return NextState;            }

    }

    public void move(String direction, double speed) {
        switch (direction) {
            case "forward":
                //robot moves backwards
                leftFront.setPower(speed);
                rightFront.setPower(-speed);
                leftBack.setPower(speed);
                rightBack.setPower(-speed);
                break;
            case "backward":
                //robot moves forwards
                leftFront.setPower(-speed);
                rightFront.setPower(speed);
                leftBack.setPower(-speed);
                rightBack.setPower(speed);
                break;
            case "right":
                //robot strafes right
                leftFront.setPower(-speed);
                rightFront.setPower(-speed);
                leftBack.setPower(speed);
                rightBack.setPower(speed);
                break;
            case "left":
                //robot strafes left
                leftFront.setPower(speed);
                rightFront.setPower(speed);
                leftBack.setPower(-speed);
                rightBack.setPower(-speed);
                break;
            case "ccw":
                //robot turns clockwise(to the right)
                leftFront.setPower(-speed);
                rightFront.setPower(-speed);
                leftBack.setPower(-speed);
                rightBack.setPower(-speed);
                break;
            case "cw":
                //robot turns counterclockwise(to the left)
                leftFront.setPower(speed);
                rightFront.setPower(speed);
                leftBack.setPower(speed);
                rightBack.setPower(speed);
                break;
        }
    }

}
