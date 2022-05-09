#include <iostream>
#include <fstream>
#include <math.h>
#include <string.h>
#include <stdio.h>

using namespace std;

#define PI 3.14159265

class Point {
  public:
    double x;
    double y;
	Point(double _x, double _y){
		x=_x;
		y=_y;
	}
	Point(){
		x=0;
		y=0;
	}
	bool Equals(Point other){
		return (x==other.x && y == other.y);
	}
};

class ConnectedPoints {
  public:
    Point top;
    Point bottom;
	ConnectedPoints(Point p1, Point p2){
		if (p1.y>p2.y){
			top = p2;
      		bottom = p1;
		}
		else {
			top = p1;
      		bottom = p2;
		}
    }
	double squaredLength(){
		double x_d = (top.x-bottom.x);
		double y_d = (top.y-bottom.y);
		return x_d * x_d + y_d * y_d;
	}
	double lineAtY(double y) {
		if (bottom.y-top.y != 0)
			return (y-top.y)*(bottom.x-top.x)/(bottom.y-top.y)+top.x;
		else 
			return (top.x + bottom.x)/2.0;
	}
	bool verticallyIntersects(ConnectedPoints other){
		return (bottom.y > other.top.y) && (other.bottom.y > top.y);
	}
};

Point getTargetPoint(ConnectedPoints c1, ConnectedPoints c2);
Point getTargetPoint(ConnectedPoints c0);

double max(double a, double b){
	if (a>b)
		return a;
	else
		return b;
}

double min(double a, double b) {
	if (a<b)
		return a;
	else
		return b;
}

double getAngle(Point target){
	double angle = (atan2(target.y - 51, target.x - 40) - atan2(0 - 51, 40 - 40)) * 180 / PI;
	if (angle < -180) angle += 360;
	// Currently steering right is negative and steering left is positive
	return -angle;
}

Point getTargetPoint(ConnectedPoints c1, ConnectedPoints c2){

	if (c1.bottom.y == c1.top.y || c2.bottom.y == c2.top.y)
		return Point(39,25);

	ConnectedPoints R = c2;
	ConnectedPoints L = c1;
	
	if (c1.bottom.x > c2.bottom.x) {
		R = c1;
		L = c2;
	}

	if ((!R.verticallyIntersects(L)) || (R.bottom.Equals(L.top) || R.top.Equals(L.bottom))){
		if (L.squaredLength() > R.squaredLength())
			return getTargetPoint(L);
		else
			return getTargetPoint(R);
	}
	
	double By = min(R.bottom.y, L.bottom.y);
	double Bx = (L.lineAtY(By) + R.lineAtY(By))/2.0;

	double Ty = max(R.top.y, L.top.y);
	double Tx = (R.lineAtY(Ty) + L.lineAtY(Ty))/2.0;

	return Point((Tx+Bx)/2.0,(Ty+By)/2.0);
}

Point getTargetPoint(ConnectedPoints c0) {

	Point p2;
	Point p3;

	if (c0.bottom.x < 39){
		p2.x = 78;
		p2.y = 0;
		p3.x = 78;
		p3.y = 51;
	}
	else {
		p2.x = 0;
		p2.y = 0;
		p3.x = 0;
		p3.y = 51;
	}

	ConnectedPoints c1(p2, p3);

	return getTargetPoint(c0, c1);
}

int main(int argc, char **argv)
{
	if (argc == 9)
	{
		Point p0(atof(argv[1]), atof(argv[2]));
		Point p1(atof(argv[3]), atof(argv[4]));
		Point p2(atof(argv[5]), atof(argv[6]));
		Point p3(atof(argv[7]), atof(argv[8]));

		ConnectedPoints c0(p0,p1);
		ConnectedPoints c1(p2,p3);

		Point result = getTargetPoint(c0);
		double angle = getAngle(result);
		
		cout << result.x << ", " << result.y << ", " << angle << endl;
	}
	else if (argc == 5) {
		Point p0(atof(argv[1]), atof(argv[2]));
		Point p1(atof(argv[3]), atof(argv[4]));

		ConnectedPoints c0(p0,p1);

		Point result = getTargetPoint(c0);
		double angle = getAngle(result);
		
		cout << result.x << ", " << result.y << ", " << angle << endl;
	}
	else {
		Point p0(0, 5);
		Point p1(19, 4);
		Point p2(62, 4);
		Point p3(78, 4);

		ConnectedPoints c0(p0,p1);
		ConnectedPoints c1(p2,p3);

		Point result = getTargetPoint(c0);
		double angle = getAngle(result);
		
		cout << result.x << ", " << result.y << ", " << angle << endl;
	}

	return 0;
}
