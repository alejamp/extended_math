import 'package:extended_math/extended_math.dart';

List<Vector> pointList1 = <Vector>[
        Vector(<double>[86, 3]),
        Vector(<double>[262, 7]),
        Vector(<double>[72, 12]),
        Vector(<double>[233, 14]),
        Vector(<double>[222, 16]),
        Vector(<double>[242, 19]),
        Vector(<double>[174, 21]),
        Vector(<double>[199, 22]),
        Vector(<double>[210, 23]),
        Vector(<double>[245, 27]),
        Vector(<double>[223, 28]),
        Vector(<double>[171, 29]),
        Vector(<double>[213, 32]),
        Vector(<double>[206, 34]),
        Vector(<double>[158, 36]),
        Vector(<double>[215, 36]),
        Vector(<double>[194, 40]),
        Vector(<double>[155, 43]),
        Vector(<double>[390, 145]),
        Vector(<double>[325, 151]),
        Vector(<double>[430, 165]),
        Vector(<double>[569, 166]),
        Vector(<double>[548, 171]),
        Vector(<double>[486, 172]),
        Vector(<double>[585, 174]),
        Vector(<double>[513, 175]),
        Vector(<double>[581, 178])  
];

List<Vector> pointList2 = <Vector>[
        Vector(<double>[94, 3]),
        Vector(<double>[129, 10]),
        Vector(<double>[135, 6]),
        Vector(<double>[100, 16]),
        Vector(<double>[88, 18]),
        Vector(<double>[109, 22]),
        Vector(<double>[35, 23]),
        Vector(<double>[63, 24]),
        Vector(<double>[75, 25]),
        Vector(<double>[112, 30]),
        Vector(<double>[89, 31]),
        Vector(<double>[32, 31]),
        Vector(<double>[78, 35]),
        Vector(<double>[70, 37]),
        Vector(<double>[19, 38]),
        Vector(<double>[80, 39]),
        Vector(<double>[58, 43]),
        Vector(<double>[15, 46]),
        Vector(<double>[259, 151]),
        Vector(<double>[194, 158]),
        Vector(<double>[299, 171]),
        Vector(<double>[433, 171]),
        Vector(<double>[414, 176]),
        Vector(<double>[354, 177]),
        Vector(<double>[449, 178]),
        Vector(<double>[380, 180]),
        Vector(<double>[445, 183])
  ];

  List<List<double>> expectedValues = 
  <List<double>>[
       <double>[0.60628712500923021,     0.00059969215221173516, -85.656775800903588],
       <double>[0.010863088422024825,    0.58853684011367191,     -1.6919055825149059],
       <double>[0.000088084825486304467, 0.000063754043404499572,  0.53717560168513312]
  ];

void main() {
  testTransformation();


  // print("Hola!");

  // var expected = Matrix(expectedValues);

  // List<int> sample = <int>[9,5,13,21];
  // RANSAC.homography(sample, pointList1, pointList2);

  var estimator = new RansacHomographyEstimator(0.001, 0.90);
  var res = estimator.estimate(pointList1, pointList2);
  
  print("Min distance:" + RansacTools.mindiff.toString());
  
  if (res == null) {
    print("FAIL!");
    return;
  }

  printMatrix(res);  
  
}

void testTransformation() {
  var m = Matrix(
    <List<double>>[
      <double>[2,-1,1],
      <double>[0,-2,1],
      <double>[1,-2,0],
    ]
  );

  var v = Vector(<double>[1,2]);

  var res = RansacTools.transform(m, v);

  printVector(res);
}


void printVector(Vector v) {
  print("x: " + v.data[0].toStringAsFixed(3) + ", y: " + v.data[1].toStringAsFixed(3));
}

void printMatrix(Matrix m) {
  print("-------------------------------------------------");
    for (int i = 0; i < m.rows; i++){ 
      String l = '';
      for (int j = 0; j < m.columns; j++){ 
        l = l + m.itemAt(i+1, j+1).toStringAsFixed(4) + ",  ";
      }
      print(l);
    }
  print("-------------------------------------------------");  
}