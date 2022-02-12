package org.firstinspires.ftc.teamcode.vision;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Arrays;

public class SimpleVisionYCbCr extends OpenCvPipeline {
    Telemetry telemetry = null;

    volatile boolean[] positions = {false,false,false};

    public Scalar x = new Scalar(0,0,0,0), low, high, low2, high2;
    private final boolean[] left = {true, false, false}, middle = {false, true, false}, right = {false, false, true}, failsafe = right;

    public final Scalar[] blueColor = {new Scalar(0,101,140,0), new Scalar(255,125,175,255)};
    public final Scalar[] redColor = {new Scalar(0,172,98,0), new Scalar(255,196,116,255)};
    public final Scalar[] capstoneColor = {new Scalar(0,80,75,0), new Scalar(255,95,107,255)};

    public SimpleVisionYCbCr(Telemetry telemetry, Scalar[] tapeColor, Scalar[] capstoneColor) {
        this.telemetry = telemetry;
        low = tapeColor[0];
        high = tapeColor[1];
        low2 = capstoneColor[0];
        high2 = capstoneColor[1];
    }

    public SimpleVisionYCbCr(Telemetry telemetry) {
        this.telemetry = telemetry;
        low = redColor[0];
        high = redColor[1];
        low2 = capstoneColor[0];
        high2 = capstoneColor[1];
    }

    @Override
    public Mat processFrame(Mat input) {
        // YCbCr scalars
        ArrayList<VisionObject> capstonePotential = DetectionMethods.detectYCrCb(input, low2, high2, 0.05,
                0.95,0,1,0.1,0.25,"capstone"); // detect the capstone
        ArrayList<VisionObject> capstone = new ArrayList<>();
        for(VisionObject v : capstonePotential) {
            capstone.add(v);
            Imgproc.circle(input,new Point(v.x,v.y), (int) v.magSize(),new Scalar(0,255,0,0),5);
            if(telemetry != null) {
                telemetry.addLine(v.toString());
            }
        } // Telemetry for the driver
        double ymin, ymax;
        if(capstone.size() > 0) {
            // Calculate minimum and maximum bounds for height of the tape
            ymin = capstone.get(0).y/input.height();
            ymax = (capstone.get(0).y+capstone.get(0).ysize*2)/input.height();
        } else {
            // In case the capstone wasn't found
            ymin = 0.6;
            ymax = 1;
        }
        ArrayList<VisionObject> tapePotential = DetectionMethods.detectYCrCb(input, low, high, 0.05,
                1,ymin, ymax,0.03,0.12,"tape"); // Detect the tape
        ArrayList<VisionObject> tape = new ArrayList<>();
        for(VisionObject v : tapePotential) {
            if(capstone.size() < 1 || !(Math.abs(v.x-capstone.get(0).x)/input.width() < 0.1)) {
                tape.add(v);
                Imgproc.circle(input,new Point(v.x,v.y), (int) v.magSize(),new Scalar(255,0,0,0),5);
                if(telemetry != null) {
                    telemetry.addLine(v.toString());
                }
            }
        }

        // Tape counting
        if(capstone.size() == 1 && tape.size() == 2) {
            assert telemetry != null;
            telemetry.addLine("Hybrid capstone and tape detection");
            int leftTape = 0, rightTape = 0;
            for(VisionObject t : tape) { // Count the number of pieces of tape on each side of the capstone
                if(t.x < capstone.get(0).x) {
                    leftTape++;
                } else {
                    rightTape++;
                }
            }
            if(leftTape > rightTape) {
                positions = right; // Case right capstone
            }else if(leftTape == rightTape) {
                positions = middle; // Case middle capstone
            }else {
                positions = left; // Case left capstone
            }
        }
        // Second priority capstone location
        else if(capstone.size() == 1) {
            assert telemetry != null;
            telemetry.addLine("Capstone-only detection");
            if((double)(capstone.get(0).x/input.width())<2.0/5) { // If capstone is in the left portion of the screen
                positions = left;
            } else if((double)(capstone.get(0).x/input.width())>3.0/5) { // If capstone is in the right portion of the screen
                positions = right;
            } else { // If capstone is in the middle of the screen
                positions = middle;
            }
        }
        // Third priority tape location
        else {
            assert telemetry != null;
            telemetry.addLine("Tape-only detection");
            positions = new boolean[]{true,true,true};
            for(VisionObject t : tape) {
                if((double)(t.x/input.width())<2.0/5) { // If tape is found in the left portion of the screen, the capstone cannot be there
                    positions[0] = false;
                } else if((double)(t.x/input.width())>3.0/5) { // Same as above, but with right
                    positions[2] = false;
                } else { // Same as above, by with middle
                    positions[1] = false;
                }
            }
            int i = 0;
            for(boolean p : positions) { // Count the number of places a capstone could be
                if(p) { i++; }
            }
            if(i >= 2) {
                positions = failsafe; // failsafe in case it's ambiguous
                telemetry.addLine("Detection failed");
            }
            if(Arrays.equals(positions, new boolean[]{false, false, false})) {
                positions = failsafe; // failsafe in case it's ambiguous
                telemetry.addLine("Detection failed");
            }
        }

        assert telemetry != null;
        telemetry.addLine(Arrays.toString(positions));

        Imgproc.cvtColor(input,input,Imgproc.COLOR_RGB2YCrCb);
        if(telemetry != null) {
            telemetry.update();
        }
        return input;
    }

    public boolean[] getPositions() {
        return positions;
    }
}
