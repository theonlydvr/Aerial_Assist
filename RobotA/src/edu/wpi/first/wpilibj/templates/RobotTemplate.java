package edu.wpi.first.wpilibj.templates;

import edu.wpi.first.wpilibj.AnalogChannel;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Relay.Value;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.SimpleRobot;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.templates.BadAssController.AxisType;
import edu.wpi.first.wpilibj.templates.BadAssController.ButtonType;

/*
 * This is the main class for the robot from 2014. It has code to interact with all the sensors and motors on the robot and some methods for automation
 */

public class RobotTemplate extends SimpleRobot {
    /*
     * Class for interacting with the tilting platform
     */
    class Platform extends Talon 
    {
        AnalogPotentiometer pot; // the potentiometer which provided an angle reading to adjust the platofrm
        
        /*
         * Takes the channel on the electronics board and a constructed potentiometer (which is done using a different channel) to create a Platform object
         */
        public Platform(int channel, AnalogPotentiometer pot) {
            super(channel);
            this.pot = pot;
        }
        
        
        /*
         * Asjusts the angle of the Platform until it is at the desired value
         */
        public void setAngle(double angle)
        {
            set(angle < pot.get() ? -0.4 : 0.4); // fancy values that were found by testing
            long startTime = System.currentTimeMillis();
            while (Math.abs(angle - Math.abs(pot.get())) > 0 && System.currentTimeMillis() - startTime < 3000){}
            set(0);
        }
        
        /*
         * Gets the angle from the potentiometer, pretty simple
         */
        public double getAngle()
        {
            return pot.get();
        }
    }
    
    /*
     * Class used to interact with the shooting mechanism
     */
    class Shooter extends Talon
    {
        public Shooter(int channel) {
            super(channel);
        }
        
        /*
         * Pulled the shooter back until the mechanism hit the limit switch signaling that more pulling would cause it to fire
         */
        public void load()
        {
            shooter.set(0.5);
            /*
            while (!shooterStop.get()){}
            shooter.set(0);
            */
            Timer.delay(2);
            shooter.set(0);
        }
        
        /*
         * After being loaded would pull the mechanism the last little bit in order to fire
         */
        public void shoot()
        {
            //double angle = aim();
            //platform.set(angle < 0 ? -0.5 : 0.5);
            //while (Math.abs(platformAngle.get()) < angle ) {}
            //platform.set(0);
            shooter.set(0.5);
            Timer.delay(1);
            shooter.set(0);
        }
        
        
        /*
         * Fancy function meant to be used during autonomous to calculate how the robot should aim and fire the ball, was never fully tested or used
         */
        public void aim()
        {    
            double Xt = ultrasonic.getDistance();
            double Vi = 7.668;
            double Yt = 0; //tbd
            double g = 9.8;
            double K1 = Xt * Xt;
            double K2 = 2 * Vi * Vi * Xt / g;
            double K3 = 2 * Vi * Vi * Yt / g;
            double aMax = 90; //tbd
            double aMin = 0; //tbd
            double aMid = (aMax + aMin) / 2;
            double vMax = K2 * Math.sin(aMax)*Math.cos(aMax) - K3 * Math.cos(aMax) * Math.cos(aMax) - K1;
            double vMin = K2 * Math.sin(aMin)*Math.cos(aMin) - K3 * Math.cos(aMin) * Math.cos(aMin) - K1;
            double vMid = K2 * Math.sin(aMid)*Math.cos(aMid) - K3 * Math.cos(aMid) * Math.cos(aMid) - K1;
            int skip;
            
            while ( vMid > 0.05 || vMid < -0.05)
            {
                //variable to check if values have been changed
                skip = 0;
                //recalculating middle angle
                aMid = (aMax + aMin) / 2;
                //recalculating value of the function for the middle angle
                vMid = K2 * Math.sin(aMid)*Math.cos(aMid) - K3 * Math.cos(aMid) * Math.cos(aMid) - K1;
                //checking if the robot is too far away by checking if the middle angle value 
                //is further from zero than a loop before
            
                if (vMid < 0)
                {
                //Checking if zero is between middle angle and minimum angle
                if (vMin > 0)
                {
                    //decreasing search area accordingly
                    aMax = aMid;
                    vMax = vMid;
                    skip = 1;
                }
                //Checking if zero is between middle angle and maximum angle
                else if (vMax > 0)
                {
                    //decreasing search area accordingly
                    aMin = aMid;
                    vMin = vMid;
                    skip = 1;
                }
                }
                if ((skip==0)&&(vMid > 0))
                {
                    //Checking if zero is between middle angle and minimum angle
                    if (vMin < 0)
                    {
                    //decreasing search area accordingly
                    aMax = aMid;
                    vMax = vMid;
                    skip = 1;
                }
                //Checking if zero is between middle angle and maximum angle
                else if (vMax < 0)
                {
                    //decreasing search area accordingly
                    aMin = aMid;
                    vMin = vMid;
                    skip = 1;
                }
                }
                //Checking if zero is on middle angle
                if ((skip==0)&&(vMid == 0))
                {
                    //setting everything to middle angle
                    aMax = aMid;
                    aMin = aMid;
                    vMax = vMid;
                    vMin = vMid;
                    skip = 1;
                }
                if ((skip==0)&&(((vMid > 0) && (vMax > 0) && (vMin > 0)) ||((vMid < 0) && (vMax < 0) && (vMin < 0))))
                {
                //Exception where both zeroes are between two angles
                //seeing if all values are positive
                if (vMid > 0)
                {
                    //Checking if vMax is further away from zero than vMin
                    if (vMax > vMin)
                    {
                        //lowering search area accordingly
                        aMax = aMid;
                        vMax = vMid;
                        skip = 1;
                    }
                    else
                    {
                        aMin = aMid;
                        vMin = vMid;
                        skip=1;
                    }
                }
                //seeing if all values are negative
                else if (vMid < 0)
                {
                    //Checking if vMax is closer to zero than vMin
                    if (vMax > vMin)
                    {
                        //lowering search area accordingly
                        aMin = aMid;
                        vMin = vMid;
                        skip = 1;
                    }
                        else
                        {
                            aMax = aMid;
                            vMax = vMid;
                            skip = 1;
                        }
                    }
                }
            }
            //recalculating aMid and returning aMid
            aMid = (aMin+aMax)/2;    
            platform.setAngle(aMid);
        }
    }
    
    /*
     * A class used to interact with an Ultrasonic sensor and get values from it
     */
    class Ultrasonic extends AnalogChannel
    {
        double distance;
        public Ultrasonic(int channel) {
            super(channel);
            distance = getDistance();
        }
        
        public double getDistance()
        {
            double vcc = 5;
            double vi = vcc / 522;
            return super.getVoltage() / vi + 52;
        }
    }

    RobotDrive chassis = new RobotDrive(3, 4, 1, 2);
    Joystick leftStick = new Joystick(1);
    Joystick rightStick = new Joystick(2);
    BadAssController controlStick = new BadAssController(3);
    Relay rollers = new Relay(8);
    Talon arms = new Talon(8);
    AnalogPotentiometer platformAngle = new AnalogPotentiometer(1);
    Ultrasonic ultrasonic = new Ultrasonic(2);
    Platform platform = new Platform(6, platformAngle);
    Shooter shooter = new Shooter(5);
    DigitalInput stop90 = new DigitalInput(1); // arms at right angle
    DigitalInput stop20 = new DigitalInput(10); // arm at 20 degree angle
    DigitalInput stopLoad = new DigitalInput(9); // limit switch to signal that the shooting mechanism is loaded
    Servo leftSpeed = new Servo(7); 
    Servo rightSpeed = new Servo(9);
    
    public void robotInit()
    {
        
    }
    
    /*
     * Drives forward... NASA worthy autonomous
     */
    public void autonomous() {
        chassis.setSafetyEnabled(false);
        Timer.delay(5);
        chassis.tankDrive(1, 1);
        /*
        arms.set(-0.5);
        while (ultrasonic.getDistance() > 300)
        {
            if (stop20.get())
            {
                arms.set(0);
            }
        }
        chassis.drive(0, 0);
        shooter.aim();
        shooter.set(1);
        */
        Timer.delay(3);
        chassis.tankDrive(0, 0);
   }

    /**
     * This function is called once each time the robot enters operator control.
     */
    public void operatorControl() {
        System.out.println("hey"); // log some stuff
        chassis.setSafetyEnabled(false); 
        boolean loaded = false, releasing = false, prepPick = false, finishPick = false; // flags to signal automated functions
        long startTime = 0;
        
        while (isOperatorControl() && isEnabled())
        {
            // automatically loads the shooting mechanism if it is not releasing or already loaded
            if (controlStick.getStart() || (!loaded && !releasing))
            {
                shooter.set(-1);
            }
            else if (!releasing)
            {
                shooter.set(0);
            }
            
            loaded = stopLoad.get();
            
            // gear change
            if (leftStick.getRawButton(2) || rightStick.getRawButton(2))
            {
                if (leftSpeed.get() > 0.5 && rightSpeed.get() > 0.5)
                {
                    leftSpeed.set(0);
                    rightSpeed.set(0);
                }
                else if (leftSpeed.get() < 0.5 && rightSpeed.get() < 0.5)
                {
                    leftSpeed.set(1);
                    rightSpeed.set(1);
                }
                Timer.delay(0.5);
            }
            
            // drive controls, nice and simple
            chassis.tankDrive(-leftStick.getY(), -rightStick.getY());
            
            // move rollers forward
            if (controlStick.getButton(ButtonType.kRightStick))
            {
                rollers.set(Value.kForward);
            }
            // move rollers backwards
            else if (controlStick.getButton(ButtonType.kLeftStick))
            {
                rollers.set(Value.kReverse);
            }
            
            // if the arms are being put into a position in order to pick a ball up
            if (prepPick)
            {
                // stop the arms when they reach 20 degrees
                if (stop20.get())
                {
                    arms.set(0);
                }
                // in case the switch is not working stop after 3 seconds
                if (System.currentTimeMillis() - startTime > 3000)
                {
                    platform.set(0);
                    arms.set(0);
                    startTime = 0;
                    prepPick = false;
                }
            }
            
            // if the ball has been picked up and it has been signaled thart it should be brought in
            if (finishPick)
            {
                // stop the arms when they reach 90 degrees
                if (stop90.get())
                {
                    arms.set(0);
                }
                // in case the switch is not working stop after 3 seconds
                if (System.currentTimeMillis() - startTime > 3000)
                {
                    arms.set(0);
                    platform.set(0);
                    rollers.set(Value.kOff);
                    finishPick = false;
                    startTime = 0;
                }
            }
            
            // button input to prepare ball pick up, singlas to the dashboard to graphically display a clicked button
            if (controlStick.getAButton())
            {
                SmartDashboard.putBoolean("A", true);
                prepPick = true;
                prepPick();
                startTime = System.currentTimeMillis();
            }
            else SmartDashboard.putBoolean("A", false);
            
            // button input to finish ball pick up, singlas to the dashboard to graphically display a clicked button
            if (controlStick.getBButton())
            {
                SmartDashboard.putBoolean("B", true);
                finishPick();
                finishPick = true;
                startTime = System.currentTimeMillis();
            }
            else SmartDashboard.putBoolean("B", false);
            
            // just gives response that button is clicked
            if (controlStick.getXButton())
            {
                //finishPick();
                //Timer.delay(0.2);
                SmartDashboard.putBoolean("X", true);
            }
            else SmartDashboard.putBoolean("X", false);
            
            // turns the rollers off
            if (controlStick.getYButton())
            {
                //finishPick();
                //Timer.delay(0.2);
                SmartDashboard.putBoolean("Y", true);
                rollers.set(Value.kOff);
            }
            else SmartDashboard.putBoolean("Y", false);
            
            // manual shooting
            if (controlStick.getTrigger(GenericHID.Hand.kRight) && loaded)
            {
                shooter.set(-1);
                Timer.delay(1);
            }
            
            // release the shooting mechanism
            if (controlStick.getTrigger(GenericHID.Hand.kLeft))
            {
                shooter.set(1);
                releasing = true;
            }
            else if (releasing)
            {
                releasing = false;
                shooter.set(0);
            }
            
            // move the motors using input from the controller
            platform.set(controlStick.getAxis(AxisType.kLeftY));
            arms.set(-controlStick.getAxis(AxisType.kRightY) / 1.25);
            // log data to the dashboard
            SmartDashboard.putNumber("Ultrasonic: ", ultrasonic.getDistance() * 2.54);
            SmartDashboard.putNumber("Potentiometer: ", platformAngle.get());
            SmartDashboard.putNumber("Left Stick", leftStick.getY());
            SmartDashboard.putNumber("Right Stick", rightStick.getY());
            SmartDashboard.putNumber("Platform Stick", -controlStick.getAxis(AxisType.kLeftY));
            SmartDashboard.putNumber("Arm Stick", -controlStick.getAxis(AxisType.kRightY));
            SmartDashboard.putBoolean("Loaded", loaded);
            SmartDashboard.putBoolean("Releasing", releasing);
            SmartDashboard.putBoolean("Top Arm Stop", stop90.get());
            SmartDashboard.putBoolean("Bottom Arm Stop", stop20.get());
            SmartDashboard.putBoolean("Prep Load", prepPick);
            SmartDashboard.putBoolean("Prep Finish", finishPick);
            SmartDashboard.putBoolean("Speed Gear", rightSpeed.get() == 0.9);
        }
    }
    
    /**
     * This function is called once each time the robot enters test mode.
     */
    public void test() {
    
    }
    
    /*
     * A method used to automate the process for moving the arms down and making the rollers move in order to pick up the ball
     */
    public void prepPick()
    {
        arms.set(0.5);
        rollers.set(Value.kForward);
        platform.set(-0.5);
        
    }
    
    /*
     * Brings the ball onto the platform once it has been picked up
     */
    public void finishPick()
    {
        arms.set(-0.5);
        platform.set(0.5);
    }
    
    /*
     * Reverses the rollers and lowers the arms in order to spit the ball out, then brings the arms back to standard position
     */
    public void release()
    {
        rollers.set(Value.kReverse);
        platform.set(-0.5);
        arms.set(0.5);
        Timer.delay(1.5);
        rollers.set(Value.kOff);
        arms.set(-0.5);
        platform.set(0.5);
        Timer.delay(1);
        arms.set(0);
        platform.set(0);
    }   
}
