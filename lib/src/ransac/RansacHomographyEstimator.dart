
import 'package:extended_math/extended_math.dart';
import 'package:extended_math/src/ransac/ransac-tools.dart';
import 'package:extended_math/src/ransac/ransac.dart';

//https://github.com/accord-net/framework/blob/792015d0e2ee250228dfafb99ea0e84d031a29ae/Sources/Accord.Vision/Accord.Imaging/RansacHomographyEstimator.cs
class RansacHomographyEstimator{

  double threshold;
  double probability;
  // List<Vector> pointSet1;
  // List<Vector> pointSet2;
  List<double> d2;
  RANSAC ransac;


  RansacHomographyEstimator(double threshold, double probability ) {
      this.threshold = threshold;
      this.probability = probability;
      // // Create a new RANSAC with the selected threshold
      ransac = new RANSAC(4, threshold, probability);

      // // Set RANSAC functions
      // ransac.Fitting = homography;
      // ransac.Degenerate = degenerate;
      // ransac.Distances = distance;
  }


  Matrix estimate(List<Vector> points1, List<Vector> points2) {
      // Initial argument checks
      if (points1.length != points2.length)
          throw ("The number of points should be equal.");
        
      if (points1.length < 4)
          throw ("At least four points are required to fit an homography");      

      // Normalize each set of points so that the origin is
      //  at centroid and mean distance from origin is sqrt(2).
      SquareMatrix t1, t2;

      var norm1 =  RansacTools.normalize(points1);
      var norm2 =  RansacTools.normalize(points2);
      var p1 = norm1.points;
      var p2 = norm2.points;
      t1 = norm1.transformation;
      t2 = norm2.transformation;

      d2 = [];
      for (var e in p1)
        d2.add(0.0);

      //https://github.com/accord-net/framework/blob/792015d0e2ee250228dfafb99ea0e84d031a29ae/Sources/Accord.Vision/Accord.Imaging/RansacHomographyEstimator.cs#L292
      
      // Compute RANSAC and find the inlier points
      // Matrix3 h = ransac.compute (points1.length, out inliers);
      RansacResults ransacResults = ransac.compute(points1.length, p1, p2);

      var H = ransacResults.model;
      var inliers =ransacResults.inliers;

      if (inliers == null || inliers.length < 4)
          return null;
          // throw ("RANSAC could not find enough points to fit an homography.");


      // Compute the final homography considering all inliers
      H = homography(inliers, p1 , p2);

      // Denormalize
      // var t2inv = t2.clone(); t2inv.invert();
      // var Hxt1 = H.clone(); Hxt1.multiply(t1);
      // var Hres = t2inv.clone();

      var res = t2.inverse().matrixProduct(H.matrixProduct(t1));

      // Hres.multiply(Hxt1);


      return res;
  }


  Matrix homography(List<int> points, List<Vector> pointSet1, List<Vector> pointSet2)
  {
      // Retrieve the original points
      
      List<Vector> x1 = [];
      List<Vector> x2 = [];
      for (var idx in points) {
        x1.add(pointSet1[idx]);
        x2.add(pointSet2[idx]);
      }
      // PointF[] x1 = this.pointSet1.Get(points);
      // PointF[] x2 = this.pointSet2.Get(points);

      // Compute the homography
      return RansacTools.homography(x1, x2);
  }

  

}