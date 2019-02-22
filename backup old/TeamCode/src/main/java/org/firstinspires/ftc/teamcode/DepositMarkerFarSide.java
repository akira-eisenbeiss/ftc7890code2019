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

@Autonomous(name="depositing marker far side", group="LinearOpMode")
public class DepositMarkerFarSide extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        //wheel motors
        leftFront = hardwareMap.dcMotor.get("left front");
        leftBack = hardwareMap.dcMotor.get("left back");
        rightFront = hardwareMap.dcMotor.get("right front");
        rightBack = hardwareMap.dcMotor.get("right back");
        //distance sensor
        rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range sensor");
        depotSensor = hardwareMap.get(ColorSensor.class, "depot sensor");
        //arm motors
        motorArm1 = hardwareMap.dcmotor.get("ARM MOTOR NAME 1");
        motorArm2 = hardwareMap.dcmotor.get("ARM MOTOR NAME 2");
        intakeMotor = hardwareMap.dcmotor.get("INTAKE MOTOR NAME");

        waitForStart();
        gyro(315);
        //turns towards wall
        while (rangeSensor.getDistance(DistanceUnit.CM) >= 20){
          move ( leftFront,  rightFront, leftBack,  rightBack,
                     FORWARD, 0.3);
                     //drives up to wall
        }
        gyro(270);
        //turns towards depot

        while(!(depotSensor.blue() > depotSensor.green()) && !(depotSensor.red() > depotSensor.green())){
          move ( leftFront,  rightFront, leftBack,  rightBack,
                     FORWARD, 0.3);
                     //drives forward until depot tape
        }
        //unfolds arm, spins intake to deposit
        motorArm1.setPower(0.5);
        sleep(2000);
        motorArm2.setPower(0.5);
        sleep(2000);
        motorIntake.setPower(0.5);
        sleep(2000);
    }
    }
}
/*
* Senario 1 [Other Class]:
*   * Drive forward until ~35cm from wall
*   DROP THE Marker (
* Senario 2:
* On the far side [This Class]
* Senario 3:[Worse Case, Supernatural Chain of Unfortune Leaves Us Here]
* https://youtu.be/mWWAZBKvizg?t=255
*/
