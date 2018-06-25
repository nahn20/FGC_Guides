package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="keyDemo", group="Test")
public class keyDemo extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    //////////// NOTES \\\\\\\\\\\\\\
    /*
    Insert useMap1.a = runtime.milliseconds(); after every use of gamepad1.a to reset the cooldown.
    Use toggleMap1.a to access whether gamepad1.a is toggled or not.
    cdCheck(useMap1.a, 1000) returns true or false based on whether gamepad1.a has been used in the last 1000 milliseconds.
    */
    toggleMap toggleMap1 = new toggleMap();
    useMap useMap1 = new useMap();
    
    toggleMap toggleMap2 = new toggleMap();
    useMap useMap2 = new useMap();
    @Override
    public void runOpMode() {

        while(!opModeIsActive()){}
        
        while(opModeIsActive()){
            updateKeys();
            telemetry.addData("a", toggleMap1.a + " " + (runtime.milliseconds() - useMap1.a));
            telemetry.addData("b", toggleMap1.b + " " + (runtime.milliseconds() - useMap1.b));
            telemetry.addData("b2", toggleMap2.b + " " + (runtime.milliseconds() - useMap2.b));
            telemetry.addData("right_stick_x_pos", toggleMap1.right_stick_x_pos + " " + (runtime.milliseconds() - useMap1.right_stick_x_pos));
            telemetry.update();
        }
    }
    public void updateKeys(){ 
        if(gamepad1.a && cdCheck(useMap1.a, 1000)){
            toggleMap1.a = toggle(toggleMap1.a);
            useMap1.a = runtime.milliseconds();
        }
        if(gamepad1.b && cdCheck(useMap1.b, 500)){
            toggleMap1.b = toggle(toggleMap1.b);
            useMap1.b = runtime.milliseconds();
        }
        if(gamepad2.b && cdCheck(useMap2.b, 500)){
            toggleMap2.b = toggle(toggleMap2.b);
            useMap2.b = runtime.milliseconds();
        }
        if(gamepad1.right_stick_x > 0 && cdCheck(useMap1.right_stick_x_pos, 700)){
            toggleMap1.right_stick_x_pos = toggle(toggleMap1.right_stick_x_pos);
            useMap1.right_stick_x_pos = runtime.milliseconds();
        }
    }
    public boolean cdCheck(double key, int cdTime){
        return runtime.milliseconds() - key > cdTime;
    }
    public boolean toggle(boolean variable){
        if(variable == true){
            variable = false;
        }
        else if(variable == false){
            variable = true;
        }
        return variable;
    }
}