package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;
import org.firstinspires.ftc.robotcontroller.external.samples.ConceptVuforiaNavigation;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector;


@Autonomous(name="IMU Test", group="LinearOpMode")
public class ImuTest extends LinearOpMode {

    BNO055IMU imu;//The object
    Orientation angles;//first set of angles
    Orientation lastAngles = new Orientation();

    //angles and lastAngles are stored in the same place, use the change in angle to determine the overall rotation with the euler angles
    Acceleration gravity;

    double globalAngle=0;
    double lastAngle;

    DcMotor leftFront, leftBack, rightFront, rightBack;

    @Override
    public void runOpMode() throws InterruptedException {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        leftFront = hardwareMap.dcMotor.get("left front");
        leftBack = hardwareMap.dcMotor.get("left back");
        rightFront = hardwareMap.dcMotor.get("right front");
        rightBack = hardwareMap.dcMotor.get("right back");

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.update();

        //angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        waitForStart();
        telemetry.addLine("hello");
        telemetry.update();
        sleep(1000);

        imu(90, "cw");

        //imu.getPosition();

    }
    public void imu(int targetHeading, String dir) {
        Orientation heading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        while (heading.firstAngle - targetHeading > 10 || targetHeading - heading.firstAngle > 10) {
            heading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            move("cw", 0.3);
            /*
            if (dir == "ccw") {
                move("ccw", 0.3);
            } else if (dir == "cw") {
                move("cw", 0.3);
            }
            */
            telemetry.addData("heading: ", heading.firstAngle);
            telemetry.addData("target", targetHeading);
            telemetry.update();
        }
        stopMove();
        telemetry.addLine("finished turning");
        telemetry.addData("heading: ", heading);
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
                case "cc":
                    //robot turns clockwise(to the right)
                    leftFront.setPower(-speed);
                    rightFront.setPower(-speed);
                    leftBack.setPower(-speed);
                    rightBack.setPower(-speed);
                    break;
                case "ccw":
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
