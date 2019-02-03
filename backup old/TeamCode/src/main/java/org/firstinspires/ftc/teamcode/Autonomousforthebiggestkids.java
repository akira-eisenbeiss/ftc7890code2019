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
/*
  7890 Space Lions 2018 "FULL AUTO FINAL"
  GOALS: lowering robot, deposit marker, parking
*/
@Autonomous(name="autonomous for big", group="LinearOpMode")
public class Autonomousforthebiggestkids extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    //YO

    DcMotor leftfront, leftback, rightfront, rightback;
    DcMotor liftmotor ;

    ModernRoboticsI2cRangeSensor rangeSensor;
    ModernRoboticsI2cGyroSensor  gyroSensor;


    @Override
    public void runOpMode() throws InterruptedException {


       // Declaring Motor Bois
        leftfront = hardwareMap.dcMotor.get("left front");
        rightfront = hardwareMap.dcMotor.get("right front");
        leftback = hardwareMap.dcMotor.get("left back");
        rightback = hardwareMap.dcMotor.get("right back");
        liftmotor = hardwareMap.dcMototr.get("lift motor");

       // Declaring sensor Bois
        gyroSensor = hardwareMap.ModernRoboticsI2GyroSensor.get("gyro sensor");
        rangeSensor = hardwareMap.ModernRoboticsI2cRangeSensor.get("range sensor");


    }
    public void dropBot() {

        int groundDistance = rangeSensor.getDistance(DistanceUnit.CM);
        while (groundDistance > 3.3) {
            double landingspeed = 0.5;
            liftmotor.setpower(landingspeed)
        }
        if (distancefromground <= 2.8 CM)
        liftmotor.setPower(0);
        }

        
    }
}
