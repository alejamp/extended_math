import 'dart:math';

import 'package:extended_math/extended_math.dart';

class RansacTools {

    static double EPSILON = 1.1920929e-07;
    static double _SQRT2 = 1.4142135623730951;
    static double mindiff = double.maxFinite;

    static List<int> sample(int sampleSize, int populationSize) {
      if (sampleSize > populationSize)
          throw ("The sample size {0} must be less than the size of the population");

      List<int> idx = _sample(populationSize);     
      var res = List<int>(); 
      for (int i = 0; i < sampleSize; i++) 
        res.add(idx[i]);

      return res;
    }

    static List<int> _sample(int size)
    {
      var random = Random();
      List<int> idx = []; //VectorTools.Range(size);
      for (var i=0; i<size; i++)
        idx.add(i);

      var x = List<double>(idx.length);

      var arr = List<VectorSortCouple>(size);

      for (int i = 0; i < x.length; i++) {
          arr[i] = new VectorSortCouple(x: random.nextDouble(), idx: i);
      }

      arr.sort((a,b){
        return ((a.x - b.x)*10.0).round();
      });

      List<int> res = [];
      for (var v in arr)
        res.add(v.idx);

      return res;
  }


    static Vector transform2(Vector arg) {

    }
  
   /// <summary>
    ///   Compute inliers using the Symmetric Transfer Error,
    /// </summary>
    /// 
    /// https://github.com/accord-net/framework/blob/792015d0e2ee250228dfafb99ea0e84d031a29ae/Sources/Accord.Vision/Accord.Imaging/RansacHomographyEstimator.cs#L322
    static List<int> distance(SquareMatrix H, double t, List<Vector> pointSet1, List<Vector> pointSet2)
    {
        // Compute the projections (both directions)
        var Hinv = InvertMatrix3x3(H);//.inverse();
        var d2 = new List<double>(pointSet1.length);

        //PointF[] p1 = H.Transform(pointSet1);
        List<Vector> p1 = transformPoints(pointSet1, H);
        // for (var p in pointSet1) {
        //   // var np = p.toMatrix().matrixProduct(H).toVector();
        //   var np = transform(H, p);
        //   p1.add(np);
        // }

        // PointF[] p2 = H.Inverse().TransformPoints(pointSet2);

        List<Vector> p2 = transformPoints(pointSet2, Hinv);
        // for (var p in pointSet1) {
        //   // var np = p.toMatrix().matrixProduct(Hinv).toVector();
        //   var np = transform(Hinv, p);
        //   p2.add(np);
        // }
        
        

        // Compute the distances
        for (int i = 0; i < pointSet1.length; i++)
        {
            // Compute the distance as
            double ax = pointSet1[i].data[0] - p2[i].data[0];
            double ay = pointSet1[i].data[1] - p2[i].data[1];
            double bx = pointSet2[i].data[0] - p1[i].data[0];
            double by = pointSet2[i].data[1] - p1[i].data[1];
            d2[i] = (ax * ax) + (ay * ay) + (bx * bx) + (by * by);
        }

        // Find and return the inliers
        // return Matrix.Find(d2, z => z < t);
        List<int> res = []; 
        int idx = 0; 
        
        for (var dd in d2) {
          if(d2[idx] < mindiff) {
            mindiff = d2[idx];
            //  print("New min:" + mindiff.toString());
          }
          if (d2[idx] < t) res.add(idx);
          idx++;
        }
        return res;
    }    


    static bool collinear(Vector pt1, Vector pt2, Vector pt3)
    {
      return(
        (pt1.data[1] - pt2.data[1]) * pt3.data[0] +
        (pt2.data[0] - pt1.data[0]) * pt3.data[1] +
        (pt1.data[0] * pt2.data[1] - pt1.data[1] * pt2.data[0])).abs() < EPSILON;
    }

    /// <summary>
    ///   Normalizes a set of homogeneous points so that the origin is located
    ///   at the centroid and the mean distance to the origin is sqrt(2).
    /// </summary>
    static NormalizeResult normalize(List<Vector> points)
    {
        int n = points.length;
        double xmean = 0, ymean = 0;
        for (int i = 0; i < points.length; i++)
        {
            xmean += points[i].data[0];
            ymean += points[i].data[1];
        }
        xmean /= n;
        ymean /= n;


        double scale = 0;
        for (int i = 0; i < points.length; i++)
        {
            double x = points[i].data[0] - xmean;
            double y = points[i].data[1] - ymean;

            scale += sqrt(x * x + y * y);
        }

        scale = _SQRT2 * n / scale;


        List<List<double>> transformation = 
        [
            [scale,       0,  -scale * xmean] ,
            [0,       scale,  -scale * ymean] ,
            [0,           0,            1   ]    
        ];
        var transformationMatrix = SquareMatrix(transformation);

        List<Vector> newPoints = transformPoints(points, transformationMatrix);
        // for (Vector v in points) {
        //   // var vtoM = v.toMatrix();
        //   // var vxm = transformationMatrix.matrixProduct(vtoM);
        //   // var tv = vxm.toVector();
        //   newPoints.add(transform(transformationMatrix, v));
        // }

        return NormalizeResult(
          points: newPoints,
          transformation: transformationMatrix
        );
    }

    static List<Vector> transformPoints(List<Vector> points, Matrix t)
    {

        var elements = t.toList();
        List<Vector> r = [];
        // for (int j = 0; j < points.length; j++)
        //   r.add(Vector(<double>[0.0, 0.0]));

        for (int j = 0; j < points.length; j++)
        {
            double w = elements[6] * points[j].data[0] + elements[7] * points[j].data[1] + 1.0;
            var x = (elements[0] * points[j].data[0] + elements[1] * points[j].data[1] + elements[2]) / w;
            var y = (elements[3] * points[j].data[0] + elements[4] * points[j].data[1] + elements[5]) / w;
            // print('x: $x, y: $y');
            r.add(Vector([x,y]));
        }
 
        return r;
    }


    static Vector transform(Matrix t, Vector v) {
      var columnVector = Matrix([
        [v.data[0]],
        [v.data[1]],
        [0]
      ]);

      var txv = t.matrixProduct(columnVector);
      return Vector([txv.itemAt(1, 1), txv.itemAt(2, 1)]);
    }


    //https://github.com/accord-net/framework/blob/792015d0e2ee250228dfafb99ea0e84d031a29ae/Sources/Accord.Imaging/Tools.cs#L198
    static Matrix homography(List<Vector> points1, List<Vector> points2)
    {
            // Initial argument checks
            if (points1.length != points2.length)
                throw("The number of points should be equal.");

            if (points1.length < 4)
                throw("At least four points are required to fit an homography");


            int N = points1.length;

            SquareMatrix t1, t2; // Normalize input points
            var n1 = normalize(points1);
            var n2 = normalize(points2);
            var p1 = n1.points; 
            var p2 = n2.points;
            t1 = n1.transformation;
            t2 = n2.transformation;

            // Create the matrix A
            // var A = new List<double>(3 * N, 9);
            var A = new List<List<double>>();

            for (int i = 0; i < N*3; i++) 
              A.add(List<double>(9));


            for (int i = 0; i < N; i++)
            {
                var X = p1[i];
                double x = p2[i].data[0];
                double y = p2[i].data[1];
                int r = 3 * i;

                A[r][0] = 0;
                A[r][1] = 0;
                A[r][2] = 0;
                A[r][3] = -X.data[0];
                A[r][4] = -X.data[1];
                A[r][5] = -1;
                A[r][6] = y * X.data[0];
                A[r][7] = y * X.data[1];
                A[r][8] = y;

                r++;
                A[r][0] = X.data[0];
                A[r][1] = X.data[1];
                A[r][2] = 1;
                A[r][3] = 0;
                A[r][4] = 0;
                A[r][5] = 0;
                A[r][6] = -x * X.data[0];
                A[r][7] = -x * X.data[1];
                A[r][8] = -x;

                r++;
                A[r][0] = -y * X.data[0];
                A[r][1] = -y * X.data[1];
                A[r][2] = -y;
                A[r][3] = x * X.data[0];
                A[r][4] = x * X.data[1];
                A[r][5] = x;
                A[r][6] = 0;
                A[r][7] = 0;
                A[r][8] = 0;
            }

            var mA = Matrix(A);
            // Create the singular value decomposition
            var svd = mA.svd();
            var V = svd['rightVectors'];
                //new SingularValueDecompositionF(A, false, true, false, true);

            // List<List<double>> V = svd.RightSingularVectors;
            // MatrixN V = svd.v;


            // Extract the homography matrix
            Matrix H = Matrix(
                        [
                          [V.itemAt(1, 9) / V.itemAt(9, 9), V.itemAt(2, 9) / V.itemAt(9, 9), V.itemAt(3, 9)/ V.itemAt(9, 9)],
                          [V.itemAt(4, 9) / V.itemAt(9, 9), V.itemAt(5, 9) / V.itemAt(9, 9), V.itemAt(6, 9) / V.itemAt(9, 9)],
                          [V.itemAt(7, 9) / V.itemAt(9, 9), V.itemAt(8, 9) / V.itemAt(9, 9), V.itemAt(9, 9) / V.itemAt(9, 9)]
                        ]
                      );

            // Denormalize
            
            // H = t2.Inverse().Multiply(H.Multiply(T1));

            // var res = t2.inverse().matrixProduct(H.matrixProduct(t1));
            var t2inv = InvertMatrix3x3(t2);

            var res = Multiply3x3(t2inv, Multiply3x3(H, t1));

            // var HxT1 = H.copy();
            // HxT1.multiply(t1);

            // var t2inv = t2.clone();
            // t2inv.invert();

            // t2inv.multiply(HxT1);
            SquareMatrix sq = SquareMatrix(res.data);

            return sq;
        }


        static Matrix InvertMatrix3x3(Matrix m1)
        {
            //    m = 1 / [a(ei-fh) - b(di-fg) + c(dh-eg)]
            // 
            //                  (ei-fh)   (ch-bi)   (bf-ce)
            //  inv(A) =  m  x  (fg-di)   (ai-cg)   (cd-af)
            //                  (dh-eg)   (bg-ah)   (ae-bd)
            //
            var elements = m1.toList();

            double a = elements[0], b = elements[1], c = elements[2];
            double d = elements[3], e = elements[4], f = elements[5];
            double g = elements[6], h = elements[7];

            double m = 1.0 / (a * (e - f * h) - b * (d - f * g) + c * (d * h - e * g));
            double na = m * (e - f * h);
            double nb = m * (c * h - b);
            double nc = m * (b * f - c * e);
            double nd = m * (f * g - d);
            double ne = m * (a - c * g);
            double nf = m * (c * d - a * f);
            double ng = m * (d * h - e * g);
            double nh = m * (b * g - a * h);
            double nj = m * (a * e - b * d);

            //na, nb, nc, 
            //nd, ne, nf, 
            //ng, nh, nj
            return new Matrix([
              [na/nj, nb/nj, nc/nj],
              [nd/nj, ne/nj, nf/nj],
              [ng/nj, nh/nj, nj/nj]
            ]);
        }

        /// <summary>
        ///   Multiplies this matrix, returning a new matrix as result.
        /// </summary>
        /// 
        static Matrix Multiply3x3(Matrix m1, Matrix m2)
        {
            var elements1 = m1.toList();
            var elements2 = m2.toList();

            double na = elements1[0] * elements2[0] + elements1[1] * elements2[3] + elements1[2] * elements2[6];
            double nb = elements1[0] * elements2[1] + elements1[1] * elements2[4] + elements1[2] * elements2[7];
            double nc = elements1[0] * elements2[2] + elements1[1] * elements2[5] + elements1[2];

            double nd = elements1[3] * elements2[0] + elements1[4] * elements2[3] + elements1[5] * elements2[6];
            double ne = elements1[3] * elements2[1] + elements1[4] * elements2[4] + elements1[5] * elements2[7];
            double nf = elements1[3] * elements2[2] + elements1[4] * elements2[5] + elements1[5];

            double ng = elements1[6] * elements2[0] + elements1[7] * elements2[3] + elements2[6];
            double nh = elements1[6] * elements2[1] + elements1[7] * elements2[4] + elements2[7];
            double ni = elements1[6] * elements2[2] + elements1[7] * elements2[5] + 1.0;

            //na, nb, nc, nd, ne, nf, ng, nh, ni
            return new Matrix([
              [na/ni, nb/ni, nc/ni],
              [nd/ni, ne/ni, nf/ni],
              [ng/ni, nh/ni, ni/ni]
            ]);
        }        
  

}


class RansacResults {
  Matrix model;
  List<int> inliers;

  RansacResults({
    this.model,
    this.inliers
  });
}

class VectorSortCouple {
  double x;
  int idx;  

  VectorSortCouple({
    this.x,
    this.idx
  });
}

class NormalizeResult {
  List<Vector> points;
  SquareMatrix transformation;

  NormalizeResult({
    this.points,
    this.transformation
  });
}