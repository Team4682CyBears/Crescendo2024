// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Charged Up - 2023
// File: DebugUtils.java
// Intent: Forms util class of methods that are commonly used to help troubleshoot other code.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.common;

import java.util.*;

public class DebugUtils {

    /**
     * Method to help provide debug info for measurement discontinuities
     * @param measurements array of measurements - must be minimul length of 1, should ideally give 4+ measurements
     * @param doDescriptivePrint
     * @return true when a measurement discontinuity was found, else false
     */
    public static boolean hasMeasurementContinuity(
        ArrayList<Double> measurements,
        boolean doDescriptivePrint)
    {
        // dupe the inbound list - make sure we do deep copy
        ArrayList<Double> sortableList = new ArrayList<Double>();
        for(int inx = 0; inx < measurements.size(); ++inx)
        {
            sortableList.add((double)measurements.get(inx));
        }

        // sort the dup'd list
        Collections.sort(sortableList);

        // get the interquartile range measurements
        int firstQuartileIndex = measurements.size() / 4;
        int thirdQuartileIndex = measurements.size() * 3 / 4;
        double firstQuartile = measurements.get(firstQuartileIndex);
        double thirdQuartile = measurements.get(thirdQuartileIndex);
        double innerQuartileRange = thirdQuartile - firstQuartile;
        double lowerFence = firstQuartile - (1.5 * innerQuartileRange);
        double upperFence = thirdQuartile + (1.5 * innerQuartileRange);

        boolean measurementContinuity = true;

        // flow through original array looking for outliers
        for(int inx = 0; inx < measurements.size(); ++inx)
        {
            double nextMeasurement = measurements.get(inx);
            measurementContinuity &= (nextMeasurement >= lowerFence && nextMeasurement <= upperFence);
            if(!measurementContinuity)
            {
                break;
            }
        }

        if(doDescriptivePrint && !measurementContinuity)
        {
            System.out.println("DISCONTINUITY FOUND:");
            // flow through original array looking for outliers
            for(int inx = 0; inx < measurements.size(); ++inx)
            {
                double nextMeasurement = measurements.get(inx);
                if(nextMeasurement < lowerFence)
                {
                    System.out.println(nextMeasurement + " <- LOW OUTLIER");
                }
                else if(nextMeasurement > upperFence)
                {
                    System.out.println(nextMeasurement + " <- HIGH OUTLIER");
                }
                else
                {
                    System.out.println(nextMeasurement);
                }
            }
        }

        return measurementContinuity;
    }
}
