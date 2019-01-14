package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by wenhuizhao on 1/11/19.
 */

@Autonomous(name="autonomous for quals 2.0", group="LinearOpMode")
public class AutoQuals extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    DcMotor leftFront, leftBack, rightFront, rightBack;
    DcMotor liftMotor;

    DistanceSensor distanceSensor;

    double distanceFromGround;
    double distanceCrater;
    double power = 0.3;

    boolean l;


    @Override
    public void runOpMode() throws InterruptedException {

        //MOTORS
        leftFront = hardwareMap.dcMotor.get("left front");
        leftBack = hardwareMap.dcMotor.get("left back");
        rightFront = hardwareMap.dcMotor.get("right front");
        rightBack = hardwareMap.dcMotor.get("right back");

        liftMotor = hardwareMap.dcMotor.get("lift motor");

        distanceSensor = hardwareMap.get(DistanceSensor.class, "distance sensor");


        waitForStart();

        while(opModeIsActive()) {
            landing();
            break;
        }

    }
    public void landing () {
        /* distanceFromGround = distanceSensor.getDistance(DistanceUnit.CM);
        liftMotor.setPower(0.3);
        sleep(3000);
        liftMotor.setPower(0.0);
        sleep(2000);
        liftMotor.setPower(0.3);
        sleep(1000);
        liftMotor.setPower(0.0);
        leftFront.setPower(0.6);
        rightFront.setPower(0.6);
        leftBack.setPower(0.6);
        rightBack.setPower(0.6);
        sleep(1000);
        liftMotor.setPower(0.3);
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
        */
        liftMotor.setPower(0.3);
        sleep(3000);
        liftMotor.setPower(0.0);
        sleep(2000);
        //strafe
        liftMotor.setPower(0.0);
        leftFront.setPower(0.4);
        rightFront.setPower(0.4);
        leftBack.setPower(-0.4);
        rightBack.setPower(-0.4);
        //turn
        leftFront.setPower(0.6);
        rightFront.setPower(0.6);
        leftBack.setPower(0.6);
        rightBack.setPower(0.6);
        sleep(1500);
        //stop and raise
        liftMotor.setPower(0.3);
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
        //turn
        sleep(1500);
        liftMotor.setPower(0.0);
        leftFront.setPower(0.3);
        rightFront.setPower(0.3);
        leftBack.setPower(0.3);
        rightBack.setPower(0.3);
        sleep(3000);
    }

}
