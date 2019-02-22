package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

@Autonomous(name="Sample Aut With Two", group="Autonomous")

public class justATest extends LinearOpMode {
    DcMotor lift;
    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftBack;
    DcMotor rightBack;
    Servo marker;
    CRServo sample;
    CRServo intake;
    DcMotor box;
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    int quarterTurn = 7; //mock turn factor that does a quarter turn at .8/.9 power***

    private static final String VUFORIA_KEY = "ATfbkf//////AAABmSdKrt64X0UDrXZRbiIdwVl9iuhdq1WQN1irJAz1O/XAe4vAgTnNCQsLzqtENwAZjOfmIvzpWoO8CD4VW6uZ6gGSYAv8gLSG4Ew+HLqDbKrN+gyhJPkxwiKDFXIHWeSNuGh3UUSKGj++8ggR9vYFTyLqXpvy2uwI+z66wWL3aPUU5KjK0N8oy5+IyddBgKGDHw2QacCqKJvMuL+VOOPNYdwKC3nQ+caRIS4gsJQwQ3FZrgY/oHgfse+vLRdoBKfhV2Pl6d2kqphlXivEWaPcvkOrpkkJvqR7aYwvkkO6Aqlph6YdLRp6okEauD6zly8s4rUqoCKmOd4cEx8TfamSqg/jhc4eRbN0koLdkOWL53nG";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the Tensor Flow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;



    boolean runOnce = false;
    public void runOpMode() {

        rightFront = hardwareMap.dcMotor.get("rightFront");
        leftFront = hardwareMap.dcMotor.get("leftFront");
        rightBack = hardwareMap.dcMotor.get("rightBack");
        leftBack = hardwareMap.dcMotor.get("leftBack");
        lift = hardwareMap.dcMotor.get("lift");
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        marker = hardwareMap.servo.get("marker");
        sample = hardwareMap.crservo.get("sample");


        initVuforia();



        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        int gold = 0;
        int silver = 0;
        int loop = 0;
        waitForStart();

        sample.setPower(-1);

        if (opModeIsActive()) {

            /** Activate Tensor Flow Object Detection. */
            if (tfod != null) {
                tfod.activate();
            }

            while (opModeIsActive()) {  //just in case
                if (runOnce == false) {
                    //        liftAut();
                    int pos = 1000;
                    boolean isSensed = false;
                    while(1<2) { //chaned
                        if (tfod != null) {
                            // getUpdatedRecognitions() will return null if no new information is available since
                            // the last time that call was made.
                            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                            if (updatedRecognitions != null) {
                                //maybe move?
                                telemetry.addData("# Object Detected", updatedRecognitions.size());
                                if (updatedRecognitions.size() == 2)
                                {
                                    int goldMineralX = -1;
                                    int silverMineral1X = -1;

                                    // This just records values, and is unchanged

                                    for (Recognition recognition : updatedRecognitions)
                                    {
                                        if (recognition.getLabel().equals(LABEL_GOLD_MINERAL))
                                        {
                                            goldMineralX = (int) recognition.getLeft();
                                        }
                                        else if (silverMineral1X == -1)
                                        {
                                            silverMineral1X = (int) recognition.getLeft();
                                        }

                                    }

                                    // If there is no gold (-1) and there two silvers (not -1) the gold
                                    // is not visible, and must be on the right

                                    if (goldMineralX == -1 && silverMineral1X != -1 || goldMineralX == silverMineral1X)  //edited for now
                                    {
                                        telemetry.addData("Gold Mineral Position", "Right");
                                        telemetry.update();
                                        pos = 1;
                                        isSensed = true;
                                    }

                                    // If you can see one gold and one silver ...

                                    else if (goldMineralX != -1 && silverMineral1X != -1)
                                    {
                                        // ... if the gold is to the right of the silver, the gold is in the center ...

                                        if (goldMineralX < silverMineral1X)
                                        {
                                            telemetry.addData("Gold Mineral Position", "Center");
                                            telemetry.addData("Gold Mineral Position", goldMineralX);
                                            telemetry.addData("Silver Mineral 1Position", silverMineral1X);


                                            telemetry.update();
                                            gold = goldMineralX;
                                            silver = silverMineral1X;
                                            pos = 0;
                                            isSensed = true;
                                        }

                                        // ... otherwise it is on the left

                                        else
                                        {
                                            telemetry.addData("Gold Mineral Position", "Left");
                                            telemetry.addData("Gold Mineral Position", goldMineralX);
                                            telemetry.addData("Silver Mineral 1Position", silverMineral1X);


                                            telemetry.update();
                                            gold = goldMineralX;
                                            silver = silverMineral1X;
                                            pos = 2;
                                            isSensed = true;
                                        }
                                    }
                                    telemetry.addData("text", "Gold Mineral Position" + goldMineralX);
                                    telemetry.addData("text", "Silver Mineral 1Position" + silverMineral1X);

                                    telemetry.update();

                                }

                                telemetry.update();
                            }
                            telemetry.addData("test", "loop**");

                            telemetry.update();
                        }
                        if(loop == 50) {
                            pos = 2;
                            isSensed=true;
                        }
                        loop++;


                    }
//                    wait(10);
//                    if(!isSensed) {
//                        telemetry.addData("Did it do the thing?", "Maybe thing...?: " + isSensed);
//                        pos = 2;
//                    }
//                    telemetry.addData("Position: " + pos, pos);
//                    telemetry.addData("Gold Mineral Position", gold);
//                    telemetry.addData("Silver Mineral 1Position", silver);
//                    telemetry.update();
//                    wait(5);
//
//                    //lift down
//                    lift.setPower(-.7);
//                    wait(26); //was 26
//                    lift.setPower(0);
//                    wait(10);
//
//                    //moves off hook and turns
//                    drive(.4, .4, .4, .4);
//                    wait(3);
//                    stopRobot();
//                    drive(.9, .9, -.9, -.9); //lander turn
//                    wait(quarterTurn);
//                    stopRobot();
//                    drive(.4, .4, .4, .4);
//                    wait(2);
//                    stopRobot(4);
//
//                    sample.setPower(1); //put phone back into position
//                    stopRobot(2);
//
//                   if(pos == 1) { // right
//                       drive(.7, .7, -.7, -.7); //turns
//                       wait(2); //=3
//                       stopRobot();
//                       drive(.8, .8, .8, .8); //moves to marker hitting position
//                       wait(9);
//
//
//                       drive(.6, .6, -.6, -.6); //moves to marker hitting position - .8 was once the value
//                       wait(3);
//                       stopRobot(2);  //different wait-time
//                       drive(-.6, -.6, .6, .6); //moves to marker hitting position - .8 was once the value
//                       wait(3);
//                       stopRobot();
//
//                       drive(-.4,-.4,.4,.4); //turns to depot
//                       wait(10);
//                       stopRobot();
//                       drive(.4,.4,.4,.4);
//                       wait(7);//time to get to depot
//                       stopRobot();
//
//                       drive(-.4,-.4,.4,.4); //turns to depot
//                       wait(2);
//                       stopRobot();
//
//                       markerAut();
//
////                       drive(-.6,-.6,.6,.6);
////                       wait(3); //slight adjustment
////                       stopRobot();
////                       drive(-.5,-.5,-.5,-.5);
////                       wait(6);
////                       stopRobot();
////                       drive(.5,.5,-.6,-.6);
////                       wait(2); //slight adjustment
////
////                       drive(-.7,-.7,-.8,-.8);
////                       wait(19);
//                       stopRobot();
//                   } else if(pos == 2) { // left
//                       drive(-.7, -.7, .7, .7); //turns
//                       wait(2); //=3
//                       stopRobot();
//                       drive(.6, .6, .6, .6); //moves to marker hitting position - .8 was once the value
//                       wait(10);
//                       stopRobot();
//
//                       drive(-.6, -.6, .6, .6); //moves to marker hitting position - .8 was once the value
//                       wait(3);
//                       stopRobot(2);  //different wait-time
//                       drive(.6, .6, -.6, -.6); //moves to marker hitting position - .8 was once the value
//                       wait(3);
//                       stopRobot();
//
//                       drive(.4,.4,-.4,-.4); //turns to depot
//                       wait(10);
//                       stopRobot();
//                       drive(.4,.4,.4,.4);
//                       wait(7);//time to get to depot
//                       stopRobot();
//
//                       drive(-.6,-.6,.6,.6); //turns to depot
//                       wait(quarterTurn);
//                       stopRobot();
//
//                       markerAut();
//
////                       drive(-.6,-.6,.6,.6);
////                       wait(2); //slight adjustment
////                       stopRobot();
////                       drive(-.5,-.5,-.5,-.5);
////                       wait(7);
////                       stopRobot();
////                       drive(.6,.6,-.7,-.7);
////                       wait(2); //slight adjustment
////
////                       drive(-.7,-.7,-.8,-.8);
////                       wait(18);
//                       stopRobot();
//
//                   }else if(pos == 0){  // center
//                       drive(.7, .7, .7, .7);
//                       wait(14);
//                       stopRobot();
//
//                       drive(-.6,-.6,.6,.6); //turns to depot
//                       wait(quarterTurn - 1);
//                       stopRobot();
//
//                       markerAut();
//
////                       drive(-.7,-.7,.7,.7);
////                       wait(3); //slight adjustment
////                       stopRobot();
////                       drive(-.5,-.5,-.5,-.5);
////                       wait(6);
////                       stopRobot();
////                       drive(.6,.6,-.7,-.7);
////                       wait(5); //slight adjustment
////
////                       drive(-.7,-.7,-.8,-.8);
////                       wait(19);
//                       stopRobot();
//
////                       drive(-.5,-.5,.5,.5); //turns to depot
////                       wait(quarterTurn - 3);
////                       stopRobot();
////
////                       drive(4,4,-4,-4);
////                       wait(2);
////                       stopRobot();
////                       drive(-.7,-.7,-.7,-.7);
////                       wait(20);
////                       stopRobot();
//                   }
//                   else{
//                       telemetry.addData("Error Report", "Error, fix pos value :(");
//                   }
//
//                   //drive toward the depot before dropping off the marker
//
////                    markerAut();
////
////                    drive(.5, .5, -.5, -.5);     //moves to crator
////                    wait(1);
////                    stopRobot();
////                    drive(-.6, -.6, -.6, -.6);
////                    wait(10);
////                    stopRobot(); //safety ;3
//
//                    stopRobot();
//                    runOnce = true;
//                } else {
//                    telemetry.addData("text", "runOnce is done");
                }
            }
        }
    }
//    public void loop() {
//        telemetry.addData("text", "Lift Power: " + lift);
//    }
//
//    public void start() {
//
//        stopRobot();
//    }

    public void liftAut(){
        //for now
        lift.setPower(-.5);
        wait(26);
        lift.setPower(0);
        wait(10);
    }
    public void liftDown(){
        lift.setPower(-.7); //lift down
        wait(20);
        lift.setPower(0);
        wait(10);
    }

    public void markerAut(){

//        drive(.8,.8,-.8,-.8);
//        wait(quarterTurn - 2); //used to be -1
//        drive(-.7,-.7,.7,.7);
//        wait(4);
        stopRobot();

        marker.setPosition(1); //shacky shake - used to be 1
        wait(8);
        marker.setPosition(0); //shake - used to be 0
        wait(3);
        marker.setPosition(1); //shacky shake - used to be 1
        wait(3);
        marker.setPosition(0); //shake - used to be 0
        wait(3);
        marker.setPosition(1); //shacky shake shake - used to be 1
        wait(3);
        marker.setPosition(0); //shake - used to be 0
        wait(9);

        stopRobot();
    }

    public void stopRobot(){
        drive(0,0,0,0);
        wait(3);
    }
    public void stopRobot(int x){  //helper
        drive(0,0,0,0);
        wait(x);
    }

    public void wait(int time) {
        try {
            Thread.sleep(time * 100);//milliseconds
        } catch (InterruptedException e) {
            e.printStackTrace();
        }


    }
    public void drive(double leftFrontPower, double leftBackPower, double rightFrontPower, double rightBackPower)
    {
        leftFront.setPower(leftFrontPower);
        leftBack.setPower(leftBackPower);
        rightFront.setPower(rightFrontPower);
        rightBack.setPower(rightBackPower);

    }
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.FRONT;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }
}