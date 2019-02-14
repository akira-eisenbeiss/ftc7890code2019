package org.firstinspires.ftc.teamcode;

/**
 * Created by wenhuizhao on 12/8/18.
 */

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@Autonomous(name="gyro test", group="LinearOpMode")
public class GyroTest extends LinearOpMode {

    ModernRoboticsI2cGyro MRGyro;
    IntegratingGyroscope gyro;

    DcMotor leftFront, leftBack, rightFront, rightBack;

    public void runOpMode() {
        MRGyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro");
        gyro = (IntegratingGyroscope) MRGyro;

        leftFront = hardwareMap.dcMotor.get("left front");
        leftBack = hardwareMap.dcMotor.get("left back");
        rightFront = hardwareMap.dcMotor.get("right front");
        rightBack = hardwareMap.dcMotor.get("right back");

        MRGyro.calibrate();
        //IT TAKE 4 SECONDS TO CALIBRATE

        waitForStart();
        while(opModeIsActive()){
            gyro(270);
            gyro(90);
            int heading = MRGyro.getHeading();
            telemetry.addData("heading: ", heading);
            telemetry.update();
        }

    }
    public void gyro(int targetHeading) {
        //MRGyro.calibrate();
        int heading = MRGyro.getHeading();
            move(leftFront, leftBack, rightFront, rightBack, "CLOCKWISE", 0.5);
        telemetry.addData("heading: ", heading);
        telemetry.update();
        if (heading > targetHeading - 10 && heading < targetHeading + 10) {
            stop(leftFront, leftBack, rightFront, rightBack);
            sleep(3000);

        }

    }

    public void move(DcMotor motorlf, DcMotor motorrf, DcMotor motorlb, DcMotor motorrb,
                     String direction, double speed) {
        switch(direction) {
            case "BACKWARDS":
                //robot moves backwards
                motorlf.setPower(-speed);
                motorrf.setPower(-speed);
                motorlb.setPower(speed);
                motorrb.setPower(speed);
                break;
            case "FORWARDS":
                //robot moves forwards
                motorlf.setPower(speed);
                motorrf.setPower(speed);
                motorlb.setPower(-speed);
                motorrb.setPower(-speed);
                break;
            case "RIGHT":
                //robot strafes right
                motorlf.setPower(speed);
                motorrf.setPower(-speed);
                motorlb.setPower(speed);
                motorrb.setPower(-speed);
                break;
            case "LEFT":
                //robot strafes left
                motorlf.setPower(-speed);
                motorrf.setPower(speed);
                motorlb.setPower(-speed);
                motorrb.setPower(speed);
                break;
            case "CLOCKWISE":
                //robot turns clockwise(to the right)
                motorlf.setPower(-speed);
                motorrf.setPower(-speed);
                motorlb.setPower(-speed);
                motorrb.setPower(-speed);
                break;
            case "COUNTERCLOCKWISE":
                //robot turns counterclockwise(to the left)
                motorlf.setPower(speed);
                motorrf.setPower(speed);
                motorlb.setPower(speed);
                motorrb.setPower(speed);
                break;
        }
    }

    public void stop(DcMotor motorlf, DcMotor motorrf, DcMotor motorlb, DcMotor motorrb) {
        //robot stops moving
        motorlf.setPower(0.0);
        motorrf.setPower(0.0);
        motorlb.setPower(0.0);
        motorrb.setPower(0.0);
    }
}