/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.vision;

import static org.opencv.core.Core.mean;
import static org.opencv.imgproc.Imgproc.GaussianBlur;
import static org.opencv.imgproc.Imgproc.cvtColor;
import static org.opencv.imgproc.Imgproc.filter2D;

import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point;
import org.opencv.core.Point3;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.apriltag.AprilTagDetectorJNI;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

public class ColorDetectionPipeline extends OpenCvPipeline {

    private ArrayList<Integer> colorDetections;
    final int colorThreshold = 10;

    //RGB
//    final int[] lime = {182, 255, 0};
//    final int[] magenta = {255, 0, 198};
//    final int[] cyan = {0, 207, 255};

    //OpenCV HSV
    final int[] lime = {38, 255, 255};
    final int[] magenta = {156, 255, 255};
    final int[] cyan = {95, 255, 255};
    final int[] hues = {38, 156, 195};
    private Mat hueTargets;

    public ColorDetectionPipeline() {
        hueTargets = new Mat(3, 1, CvType.CV_32SC1);
        hueTargets.put(0, 0, 38);
        hueTargets.put(1, 0, 156);
        hueTargets.put(0, 0, 195);
    }

    @Override
    public Mat processFrame(Mat input) {
        //Crop image to center 1/9th
        Rect rectCrop = new Rect((int) (input.width() / 2), (int) (input.height() / 2), (int) (input.width() / 6), (int) (input.height() / 6));
        input = new Mat(input, rectCrop);

        //Sample center 5x5 pixels
        Rect sampleCrop = new Rect((int) (input.width() / 2), (int) (input.height() / 2), 5, 5);
        Mat colorSample = new Mat(input, sampleCrop);

        //convert image from RGB to HSV
        Mat hsvConvert = new Mat();
        cvtColor(colorSample, hsvConvert, Imgproc.COLOR_RGB2HSV);

        //Find average of each channel across sample pixels
        Mat avgColor = new Mat();
        Core.reduce(hsvConvert, avgColor, 0, Core.REDUCE_AVG);
        ArrayList<Mat> channels = new ArrayList<Mat>(3);
        Core.split(avgColor, channels);

        //Compute distance from each target color hue
        Mat hueDiff = new Mat();
        Core.absdiff(hueTargets, channels.get(0).reshape(channels.get(0).type(),3), hueDiff);

        //Find location of min hue difference
        int result = (int)Core.minMaxLoc(hueDiff).maxLoc.x;

        //Add color detection if not already included
        boolean included = false;
        for(int detection : colorDetections)
            if(result == detection) included = true;
        if(!included) colorDetections.add(new Integer(result));

        return input;
    }

    public ArrayList<Integer> getColorDetections() {
        return colorDetections;
    }

}