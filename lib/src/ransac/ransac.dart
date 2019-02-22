import 'dart:math';

import 'package:extended_math/extended_math.dart';
import 'package:extended_math/src/ransac/ransac-tools.dart';
// import 'package:homography/rhlib/tools.dart';
// import 'package:homography/rhlib/vector.dart';
// import 'package:vector_math/vector_math.dart';

class RANSAC {

    int s;
    double t;
    int trialsPerformed = 0;
    int trialsNeeded = 0;
    int maxSamplings = 100;
    int maxEvaluations = 1000;
    double probability = 0.99;
    List<double> d2 = [];

  List<int> inliers;


    //https://github.com/accord-net/framework/blob/792015d0e2ee250228dfafb99ea0e84d031a29ae/Sources/Accord.MachineLearning/Ransac.cs#L254
    RANSAC(int minSamples, double threshold, double probability)
    {
      if (minSamples < 0)
          throw("minSamples");

      if (threshold < 0)
          throw("threshold");

      if (probability > 1.0 || probability < 0.0)
          throw("Probability should be a value between 0 and 1 probability");

      this.s = minSamples;
      this.t = threshold;
      this.probability = probability;
    }

    //https://github.com/accord-net/framework/blob/792015d0e2ee250228dfafb99ea0e84d031a29ae/Sources/Accord.MachineLearning/Ransac.cs#L290
    RansacResults compute(int size,  List<Vector> pointSet1, List<Vector> pointSet2) {


      dynamic bestModel = null;
      List<int> bestInliers = [];
      int maxInliers = 0;

      int r = min(size, s);

      // For this we are going to search for random samples
      //  of the original points which contains no outliers.

      trialsPerformed = 0;              // Total number of trials performed
      trialsNeeded = maxEvaluations;    // Estimative of number of trials needed.


      // While the number of trials is less than our estimative,
      //   and we have not surpassed the maximum number of trials
      while (trialsPerformed < trialsNeeded && trialsPerformed < maxEvaluations)
      {
          // print('>>>>>>>>>>>>>>>>> trial #' + trialsPerformed.toString());
          dynamic model = null;
          List<int> sample = [];
          int samplings = 0;

          // While the number of samples attempted is less
          //   than the maximum limit of attempts
          while (samplings < maxSamplings)
          {
              // print('sample #' + samplings.toString());
              // Select at random s data points to form a trial model.
              sample = RansacTools.sample(r, size);

              // If the sampled points are not in a degenerate configuration,
              if (degenerate == null || !degenerate(sample, pointSet1, pointSet2))
              {
                  // Fit model using the random selection of points
                  // model = fitting(sample);
                  model = homography(sample, pointSet1, pointSet2);
                  
                  break; // Exit the while loop.
              }      
              samplings++; // Increase the samplings counter        
          }

          if (model == null)
            throw ("A model could not be inferred from the data points");

          
          // Now, evaluate the distances between total points and the model returning the
          //  indices of the points that are inliers (according to a distance threshold t).
          //https://github.com/accord-net/framework/blob/792015d0e2ee250228dfafb99ea0e84d031a29ae/Sources/Accord.MachineLearning/Ransac.cs#L337
          inliers = RansacTools.distance(model, t, pointSet1, pointSet2);          

          // Check if the model was the model which highest number of inliers:
          if (bestInliers == null || inliers.length > maxInliers)
          {
              // Yes, this model has the highest number of inliers.

              maxInliers = inliers.length;  // Set the new maximum,
              bestModel = model;            // This is the best model found so far,
              bestInliers = inliers;        // Store the indices of the current inliers.

              // Update estimate of N, the number of trials to ensure we pick, 
              //   with probability p, a data set with no outliers.
              double pInlier = inliers.length / size;
              double pNoOutliers = 1.0 - pow(pInlier, s);

              double nume = log(1.0 - probability);
              double den = log(pNoOutliers);
              if (den == 0)
                  trialsNeeded = nume == 0 ? 0 : maxEvaluations;
              else
                  trialsNeeded = (nume / den).round();
          }
          trialsPerformed++;
      }  
      inliers = bestInliers;
      return new RansacResults(model: bestModel, inliers: inliers);          
    }


    //https://github.com/accord-net/framework/blob/792015d0e2ee250228dfafb99ea0e84d031a29ae/Sources/Accord.Vision/Accord.Imaging/RansacHomographyEstimator.cs#L312
    static Matrix homography(List<int> points, List<Vector> pointSet1, List<Vector> pointSet2)
    {
        // Retrieve the original points
        List<Vector> x1 = []; //= pointSet1.(points);
        List<Vector> x2 = []; //= pointSet2.Get(points);
 
        String sampleStr = '';
        for (var p in points) sampleStr += '$p,';
        // print("homography sample points: " + sampleStr);

        for(var e in points) {
          x1.add(pointSet1[e]);
          x2.add(pointSet2[e]);
        }        

        // Compute the homography
        return RansacTools.homography(x1, x2);
    }

    /// <summary>
    ///   Checks if the selected points will result in a degenerate homography.
    /// </summary>
    /// 
    /// https://github.com/accord-net/framework/blob/792015d0e2ee250228dfafb99ea0e84d031a29ae/Sources/Accord.Vision/Accord.Imaging/RansacHomographyEstimator.cs#L347
    bool degenerate(List<int> points, List<Vector> pointSet1, List<Vector> pointSet2)
    {
        List<Vector> x1 = []; //= pointSet1.(points);
        List<Vector> x2 = []; //= pointSet2.Get(points);

        for(var e in points) {
          x1.add(pointSet1[e]);
          x2.add(pointSet2[e]);
        }

        // If any three of the four points in each set is collinear,
        //  the resulting homography matrix will be degenerate.

        return  RansacTools.collinear(x1[0], x1[1], x1[2]) ||
                RansacTools.collinear(x1[0], x1[1], x1[3]) ||
                RansacTools.collinear(x1[0], x1[2], x1[3]) ||
                RansacTools.collinear(x1[1], x1[2], x1[3]) ||

                RansacTools.collinear(x2[0], x2[1], x2[2]) ||
                RansacTools.collinear(x2[0], x2[1], x2[3]) ||
                RansacTools.collinear(x2[0], x2[2], x2[3]) ||
                RansacTools.collinear(x2[1], x2[2], x2[3]);
    }


 




}
