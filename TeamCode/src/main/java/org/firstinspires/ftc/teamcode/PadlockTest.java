package org.firstinspires.ftc.teamcode;

/**
 * Created by wenhuizhao on 11/8/18.
 */
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="padlock test", group="Tele Op")
public class PadlockTest extends OpMode{
    private ElapsedTime runtime = new ElapsedTime();
    CRServo padLock;
    public static int servoCntr;

    @Override
    public void init() {
        padLock = hardwareMap.crservo.get("padlock");
    }

    @Override
    public void loop() {
        boolean gamepad2A = gamepad2.a;
        boolean gamepad2B = gamepad2.b;
        boolean gamepad2X = gamepad2.x;
        //ASSIGN TO BUTTONS
       if (gamepad2B) { //close padlock
            servoCntr = 1;
        }
        else if (gamepad2A) { //open padlock
            servoCntr = 2;
        }
        else if (gamepad2X) {
           servoCntr = 0;
       }

        //CODE FOR PRESSING BUTTONS
        if(servoCntr == 0){
            padLock.setPower(0.0);
        }
        else if (servoCntr == 1){
            padLock.setPower(-1);
        }
        else if (servoCntr == 2) {
            padLock.setPower(1);
        }

        telemetry.addData("padlock", padLock.getPower());
        telemetry.addData("servo", servoCntr);
        telemetry.update();
    }




}


