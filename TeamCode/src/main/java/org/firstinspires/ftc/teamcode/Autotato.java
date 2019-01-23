package org.firstinspires.ftc.teamcode;

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

@Autonomous(name="potato auto", group="LinearOpMode")
public class Autotato extends LinearOpMode {

    DcMotor leftFront, leftBack, rightFront, rightBack;

    ModernRoboticsI2cRangeSensor rangeSensor;
    ModernRoboticsI2cGyro MRGyro;



    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    private static final String VUFORIA_KEY = "AZhcwsD/////AAABmU6nWBy9q0vlhzyw5qyhapRfHWDsE+hgriYQbJ/Ch7MUKlboZIiyGRBmc5iMadvjcwQAS2hkBE344/ZJqulkhbVXr+gekeDW70w37UFKGoeGRzxfjI/DsOWtlHx3CaAOczXgf7W5cssnDyO/zOoFiYtnRryKuouZnatP/rhl4/DHMqd1jMOWPWhI6+hNwKDdbXVdiwt5xGNTPxbyJ10jXDHt20NNGZSQry4u4/aAOS2hmrSCZgfMG6keAMB2wHj5G+dHL6AKEQwy1NHmBJZwL/O2ZlE+kkTBMSyrUNRXjkGwvSmJcdzSEDYuXGN4VJkVB/Q1CZefS3m+IkLoA75CAjnX4/buslxm3YNs+roKua3v";


    VuforiaLocalizer vuforia;

    private TFObjectDetector tfod;

    boolean runOnce = false;

    private int gold = 0;
    private int silver = 0;
    private int loop = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        leftFront = hardwareMap.dcMotor.get("left front");
        leftBack = hardwareMap.dcMotor.get("left back");
        rightFront = hardwareMap.dcMotor.get("right front");
        rightBack = hardwareMap.dcMotor.get("right back");
        //SENSORS /*
        MRGyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro");
        rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range sensor");

        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        }
        else {
            telemetry.addData("sorry", "not compatible with TFOD");
        }

        waitForStart();

    }

    public void landing() {

    }
    public void sampling() {
        while(opModeIsActive()) {
            if (tfod != null) {
                tfod.activate();
            }

            if(!runOnce) {
                int pos = 1000;
                boolean isSensed = false;
                while(!isSensed) {
                    if (tfod != null) {
                        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                        if (updatedRecognitions != null) {

                            if(updatedRecognitions.size() == 2) {
                                int goldMineralX = -1;
                                int silverMineralX = -1;

                                for(Recognition recognition : updatedRecognitions) {
                                    if(recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                        goldMineralX = (int)(recognition.getLeft());
                                    }
                                    else if (silverMineralX == -1) {
                                        silverMineralX = (int)(recognition.getLeft());
                                    }
                                }

                                if(goldMineralX == -1 && silverMineralX != -1) {
                                    telemetry.addData("gold pos", "Right");
                                    telemetry.update();
                                    pos = 1;
                                    isSensed = true;
                                }
                                else if(goldMineralX != -1 && silverMineralX != -1) {
                                    if(goldMineralX < silverMineralX) {
                                        telemetry.addData("gold pos", "Center");
                                        telemetry.update();

                                        gold = goldMineralX;
                                        silver = silverMineralX;
                                        pos = 0;
                                        isSensed = true;
                                    }
                                    else {
                                        telemetry.addData("gold pos", "Left");
                                        telemetry.update();

                                        gold = goldMineralX;
                                        silver = silverMineralX;
                                        pos = 0;
                                        isSensed = true;

                                    }

                                }

                                telemetry.addData("gold pos", goldMineralX);
                                telemetry.update();

                                if(isSensed) {
                                    pos = 2;
                                }

                                if (pos ==1) { //right
                                    gyro(45); //value for testing
                                    move(leftFront, rightFront, leftBack, rightBack, "FORWARDS", 0.3);
                                    sleep(5000);
                                }
                                else if (pos == 2) { //left

                                    gyro(225); //value for testing
                                    move(leftFront, rightFront, leftBack, rightBack, "FORWARDS", 0.3);
                                    sleep(5000);
                                }
                                else if (pos ==0) {
                                    move(leftFront, rightFront, leftBack, rightBack, "FORWARDS", 0.3);
                                    sleep(5000);
                                }
                                else {
                                    telemetry.addData("Error Report", "Error, fix pos va;ue :(");
                                }
                            }
                        }
                    }
                }
            }
        }

    }
    public void deposit() {

    }
    public void crater() {
        double distanceValue = rangeSensor.getDistance(DistanceUnit.INCH);
        while(distanceValue > 6){
            move(leftFront, leftBack, rightFront, rightBack, "FORWARDS", 0.5);
        }
        if (distanceValue <= 6){
            stop(leftFront, leftBack, rightFront, rightBack);
        }

    }
    public void nav() {

    }
    public void move(DcMotor motorlf, DcMotor motorrf, DcMotor motorlb, DcMotor motorrb,
                     String direction, double speed) {
        switch(direction) {
            case "BACK":
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
            case "TURN RIGHT":
                //robot turns clockwise(to the right)
                motorlf.setPower(-speed);
                motorrf.setPower(-speed);
                motorlb.setPower(-speed);
                motorrb.setPower(-speed);
                break;
            case "TURN LEFT":
                //robot turns counterclockwise(to the left)
                motorlf.setPower(speed);
                motorrf.setPower(speed);
                motorlb.setPower(speed);
                motorrb.setPower(speed);
                break;
        }
    }

    public void gyro(int targetHeading) {
        int heading = MRGyro.getHeading();
        move(leftFront, leftBack, rightFront, rightBack, "TURN RIGHT", 0.3);
        if (heading > targetHeading - 10 && heading < targetHeading + 10) {
            stop(leftFront, leftBack, rightFront, rightBack);
            sleep(5000);
        }
    }

    public void stop(DcMotor motorlf, DcMotor motorrf, DcMotor motorlb, DcMotor motorrb) {
        //robot stops moving
        motorlf.setPower(0.0);
        motorrf.setPower(0.0);
        motorlb.setPower(0.0);
        motorrb.setPower(0.0);
    }

    private void initVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.FRONT;

        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier( "tfodMonitorViewId", "Id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);

        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);

    }
}
