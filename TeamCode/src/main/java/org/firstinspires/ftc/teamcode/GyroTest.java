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
        //while(opModeIsActive()){
           // gyro(270);
            gyro(90, "ccw");
            int heading = MRGyro.getHeading();
            telemetry.addData("heading: ", heading);
            telemetry.update();
        //}

    }
    public void gyro(int targetHeading, String dir) {
        int heading = MRGyro.getHeading();
        while (heading < targetHeading - 10 || heading > targetHeading + 10) {
            heading = MRGyro.getHeading();

            if (dir.equals("ccw")) {
                move("ccw", 0.8);
            }
            else if (dir.equals("cw")) {
                move("cw", 0.8);
            }

            /*
            if (dir == 'L') {
                move("TURN LEFT", 0.3);
            } else if (dir == 'R') {
                move("TURN RIGHT", 0.3);
            }
            */
            telemetry.addData("heading: ", heading);
            telemetry.addData("target", targetHeading);
            telemetry.update();
        }
        stopMove();
        telemetry.addLine("done!");
        telemetry.update();

    }

    public void move(String direction, double speed) {
        switch (direction) {
            case "north":
                //robot moves backwards
                leftFront.setPower(speed);
                rightFront.setPower(-speed);
                leftBack.setPower(speed);
                rightBack.setPower(-speed);
                break;
            case "south":
                //robot moves forwards
                leftFront.setPower(-speed);
                rightFront.setPower(speed);
                leftBack.setPower(-speed);
                rightBack.setPower(speed);
                break;
            case "east":
                //robot strafes right
                leftFront.setPower(-speed);
                rightFront.setPower(-speed);
                leftBack.setPower(speed);
                rightBack.setPower(speed);
                break;
            case "west":
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
            case "north east":
                leftFront.setPower(speed);
                rightFront.setPower(0.0);
                leftBack.setPower(0.0);
                rightBack.setPower(-speed);
                break;
            case "north west":
                leftFront.setPower(0.0);
                rightFront.setPower(-speed);
                leftBack.setPower(speed);
                rightBack.setPower(0.0);
                break;
            case "south east":
                leftFront.setPower(0.0);
                rightFront.setPower(speed);
                leftBack.setPower(-speed);
                rightBack.setPower(0.0);
                break;
            case "south west":
                leftFront.setPower(-speed);
                rightFront.setPower(0.0);
                leftBack.setPower(0.0);
                rightBack.setPower(speed);
                break;
        }
    }

    public void stopMove() {
        //robot stops moving
        leftFront.setPower(0.0);
        rightBack.setPower(0.0);
        leftBack.setPower(0.0);
        rightFront.setPower(0.0);
    }
}