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
@Disabled
@Autonomous(name="vuforia test", group="LinearOpMode")
public class VuforiaTest extends LinearOpMode {


    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    private static final String VUFORIA_KEY = "AZhcwsD/////AAABmU6nWBy9q0vlhzyw5qyhapRfHWDsE+hgriYQbJ/Ch7MUKlboZIiyGRBmc5iMadvjcwQAS2hkBE344/ZJqulkhbVXr+gekeDW70w37UFKGoeGRzxfjI/DsOWtlHx3CaAOczXgf7W5cssnDyO/zOoFiYtnRryKuouZnatP/rhl4/DHMqd1jMOWPWhI6+hNwKDdbXVdiwt5xGNTPxbyJ10jXDHt20NNGZSQry4u4/aAOS2hmrSCZgfMG6keAMB2wHj5G+dHL6AKEQwy1NHmBJZwL/O2ZlE+kkTBMSyrUNRXjkGwvSmJcdzSEDYuXGN4VJkVB/Q1CZefS3m+IkLoA75CAjnX4/buslxm3YNs+roKua3v";
    VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    boolean runOnce = false;
    int pos;
    private int gold = 0;
    private int silver = 0;
    private int loop = 0;



    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        /*VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate");*/

        //Vuforia
        //initVuforia();
        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        }
        else {
            telemetry.addData("sorry", "not compatible with TFOD");
        }

        waitForStart();
        //relicTrackables.activate();
        sampling();
    }

    public void sampling() {
        if(opModeIsActive()) {
            if (tfod != null) {
                tfod.activate();
            }
            while(opModeIsActive()) {
                if (!runOnce) {
                    pos = 1000;
                    boolean isSensed = false;
                    while (1<2) {
                        if (tfod != null) {
                            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                            if (updatedRecognitions != null) {

                                telemetry.addData("# Object Detected", updatedRecognitions.size());

                                if (updatedRecognitions.size() == 2) {
                                    int goldMineralX = -1;
                                    int silverMineral1X = -1;

                                    for (Recognition recognition : updatedRecognitions) {
                                        if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                            goldMineralX = (int) (recognition.getLeft());
                                        } else if (silverMineral1X == -1) {
                                            silverMineral1X = (int) (recognition.getLeft());
                                        }
                                    }

                                    //if two silver seen, gold must be on right
                                    if (goldMineralX == -1 && silverMineral1X != -1) { //ask

                                        pos = 1;
                                        isSensed = true;

                                        telemetry.addData("gold pos", "Right");
                                        telemetry.update();
                                    }
                                    //if one gold and 1 silver
                                    else if (goldMineralX != -1 && silverMineral1X != -1) {
                                        //if gold is on right of silver, gold is in the center
                                        if (goldMineralX < silverMineral1X) {


                                            gold = goldMineralX;
                                            silver = silverMineral1X;
                                            pos = 0;
                                            isSensed = true;

                                            telemetry.addData("gold pos", goldMineralX);
                                            telemetry.addData("silver pos", silverMineral1X);
                                            telemetry.update();
                                        } else {


                                            gold = goldMineralX;
                                            silver = silverMineral1X;
                                            pos = 2;
                                            isSensed = true;

                                            telemetry.addData("gold pos", goldMineralX);
                                            telemetry.addData("silver pos", silverMineral1X);
                                            telemetry.update();

                                        }

                                    }

                                    telemetry.addData("gold pos", goldMineralX);
                                    telemetry.addData("silver pos", silverMineral1X);
                                    telemetry.addData("pos", "position is" + pos);
                                    telemetry.update();

                                    /*if (isSensed) {
                                        pos = 2;
                                    }*/


                                    if (pos == 1) { //right
                                        telemetry.addData("gold pos", "RIGHT");
                                        telemetry.update();
                                    } else if (pos == 2) { //left
                                        telemetry.addData("gold pos", "LEFT");
                                        telemetry.update();
                                    } else if (pos == 0) {
                                        telemetry.addData("gold pos", "CENTER");
                                        telemetry.update();
                                    } else {
                                        telemetry.addData("Error Report", "Error, fix pos va;ue :(");
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }

    }

    /*
     * VUFORIA Methods
     * This method initializes our VUFORIA using the license key
     * we registered on the vuforia website. The next method,
     * initTFOD, is used to init code that is needed to detect
     * the differences in gold and silver minerals.
     */
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
