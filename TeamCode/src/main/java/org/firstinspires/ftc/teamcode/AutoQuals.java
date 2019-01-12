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

@Autonomous(name="autonomous for quals", group="LinearOpMode")
public class AutoQuals extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    DcMotor leftFront, leftBack, rightFront, rightBack;
    DcMotor liftMotor;

    DistanceSensor distanceSensor;

    double distanceFromGround;

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

        }

    }
    public void landing () {
        distanceFromGround = distanceSensor.getDistance(DistanceUnit.CM);



        if (distanceFromGround <= 2.8) {
            liftMotor.setPower(0);
            sleep(4000);
            liftMotor.setPower(-0.3); //rewinds the motor so that we can retract the lift

            liftMotor.setPower(0.3);
            telemetry.addData("loop", "if");
            telemetry.addData("distance from ground", distanceFromGround);
            telemetry.update();
        }
        else {

            liftMotor.setPower(0.3);
            telemetry.addData("loop", "else");
            telemetry.addData("distance from ground", distanceFromGround);
            telemetry.update();
        }
    }
}
