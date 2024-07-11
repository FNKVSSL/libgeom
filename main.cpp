#include "geometry.hh"
#include <iostream>
#include <iomanip>

using namespace Geometry;

int main() {
    // Create a BSpline basis of degree 3 with a simple knot vector
    DoubleVector knots = {0, 0, 0, 1, 1, 1};
    BSBasis bsBasis(3, knots);

    // Print the degree of the B-spline basis
    std::cout << "Degree of B-spline basis: " << bsBasis.degree() << std::endl << std::endl;

    // Print the knot vector
    std::cout << "Knot vector: ";
    for (const auto& k : bsBasis.knots()) {
        std::cout << k << " ";
    }
    std::cout << std::endl << std::endl;

    // Set a new degree
    bsBasis.setDegree(2);
    std::cout << "New degree of B-spline basis: " << bsBasis.degree() << std::endl << std::endl;

    // Reverse the knot vector
    bsBasis.reverse();
    std::cout << "Reversed knot vector: ";
    for (const auto& k : bsBasis.knots()) {
        std::cout << k << " ";
    }
    std::cout << std::endl << std::endl;

    // Normalize the knot vector
    bsBasis.normalize();
    std::cout << "Normalized knot vector: ";
    for (const auto& k : bsBasis.knots()) {
        std::cout << std::fixed << std::setprecision(2) << k << " ";
    }
    std::cout << std::endl << std::endl;

    // Find the span for a parameter u
    double u = 0.5;
    size_t span = bsBasis.findSpan(u);
    std::cout << "Span for u = " << u << ": " << span << std::endl << std::endl;

    // Calculate basis functions for a given u
    DoubleVector coeff;
    bsBasis.basisFunctions(span, u, coeff);
    std::cout << "Basis functions for u = " << u << ": ";
    for (const auto& c : coeff) {
        std::cout << c << " ";
    }
    std::cout << std::endl << std::endl;

    // Creating control points for a B-spline curve
    PointVector controlPoints = {
        Point3D(0, 0, 0),
        Point3D(1, 2, 0),
        Point3D(2, 6, 0),
        Point3D(4, 0, 0)
    };

    // Initialize a BSCurve with these control points
    BSCurve bsCurve(controlPoints);

    // Evaluate the curve at u = 0.5
    Point3D pointAtMid = bsCurve.eval(0.5);
    std::cout << "Point on BSCurve at u = 0.5: (" << pointAtMid[0] << ", " << pointAtMid[1] << ", " << pointAtMid[2] << ")" << std::endl << std::endl;

    // Print control points of the curve
    std::cout << "Control Points of the BSCurve:" << std::endl;
    for (const auto& pt : bsCurve.controlPoints()) {
        std::cout << "(" << pt[0] << ", " << pt[1] << ", " << pt[2] << ")" << std::endl;
    }
    std::cout << std::endl;

    // Reverse the curve
    bsCurve.reverse();
    std::cout << "Control Points after reversing:" << std::endl;
    for (const auto& pt : bsCurve.controlPoints()) {
        std::cout << "(" << pt[0] << ", " << pt[1] << ", " << pt[2] << ")" << std::endl;
    }
    std::cout << std::endl;

    // Normalize the curve's basis
    bsCurve.normalize();
    std::cout << "Curve evaluation after normalization at u = 0.5:" << std::endl;
    Point3D normalizedPoint = bsCurve.eval(0.5);
    std::cout << "(" << normalizedPoint[0] << ", " << normalizedPoint[1] << ", " << normalizedPoint[2] << ")" << std::endl << std::endl;

    // Example of knot insertion
    BSCurve refinedCurve = bsCurve.insertKnot(0.5, 1);
    std::cout << "Control Points after knot insertion at u = 0.5:" << std::endl;
    for (const auto& pt : refinedCurve.controlPoints()) {
        std::cout << "(" << pt[0] << ", " << pt[1] << ", " << pt[2] << ")" << std::endl;
    }
    std::cout << std::endl;

    // Arc length estimation between parameters 0.0 and 1.0
    double arcLength = bsCurve.arcLength(0.0, 1.0);
    std::cout << "Estimated arc length of the curve: " << arcLength << std::endl << std::endl;

    // Calculate first derivative at u = 0.5
    VectorVector derivatives;
    Point3D curvePoint = bsCurve.eval(0.5, 1, derivatives);
    std::cout << "Curve point at u = 0.5: (" << curvePoint[0] << ", " << curvePoint[1] << ", " << curvePoint[2] << ")" << std::endl;
    std::cout << "First derivative at u = 0.5: (" << derivatives[1][0] << ", " << derivatives[1][1] << ", " << derivatives[1][2] << ")" << std::endl << std::endl;

    // Intersection of BSCurve with a plane defined by point and normal
    Point3D planePoint(1, 2, 0); // A point on the plane
    Vector3D planeNormal(0, 1, 0); // Normal to the plane
    DoubleVector intersections = bsCurve.intersectWithPlane(planePoint, planeNormal);
    std::cout << "Intersection parameters of BSCurve with the plane: ";
    for (const auto& param : intersections) {
        std::cout << param << " ";
    }
    std::cout << std::endl << std::endl;

    // Detailed evaluation of derivatives
    std::cout << "Detailed evaluation of derivatives at u = 0.5:" << std::endl;
    Point3D curvePointWithDerivatives = bsCurve.eval(0.5, 2, derivatives);
    std::cout << "Curve point at u = 0.5: (" << curvePointWithDerivatives[0] << ", " << curvePointWithDerivatives[1] << ", " << curvePointWithDerivatives[2] << ")" << std::endl;
    for (size_t k = 0; k < derivatives.size(); ++k) {
        std::cout << "D^" << k << " at u = 0.5: (" << derivatives[k][0] << ", " << derivatives[k][1] << ", " << derivatives[k][2] << ")" << std::endl;
    }
    std::cout << std::endl;

    // Adding more knots to refine the curve
    BSCurve moreRefinedCurve = bsCurve.insertKnot(0.25, 2);
    moreRefinedCurve = moreRefinedCurve.insertKnot(0.75, 2);
    std::cout << "Control Points after multiple knot insertions:" << std::endl;
    for (const auto& pt : moreRefinedCurve.controlPoints()) {
        std::cout << "(" << pt[0] << ", " << pt[1] << ", " << pt[2] << ")" << std::endl;
    }
    std::cout << std::endl;

    // Creating control points for a B-spline surface
    PointVector surfaceControlPoints = {
        Point3D(0, 0, 0), Point3D(1, 0, 1), Point3D(2, 0, 0),
        Point3D(0, 1, 2), Point3D(1, 1, 3), Point3D(2, 1, 2),
        Point3D(0, 2, 0), Point3D(1, 2, 1), Point3D(2, 2, 0)
    };

    // Initialize a BSSurface with these control points
    BSSurface bsSurface(2, 2, surfaceControlPoints);

    // Evaluate the surface at u = 0.5, v = 0.5
    Point3D surfacePoint = bsSurface.eval(0.5, 0.5);
    std::cout << "Point on BSSurface at (u, v) = (0.5, 0.5): (" << surfacePoint[0] << ", " << surfacePoint[1] << ", " << surfacePoint[2] << ")" << std::endl << std::endl;

    // Print control points of the surface
    std::cout << "Control Points of the BSSurface:" << std::endl;
    for (const auto& pt : bsSurface.controlPoints()) {
        std::cout << "(" << pt[0] << ", " << pt[1] << ", " << pt[2] << ")" << std::endl;
    }
    std::cout << std::endl;

    // Reverse the surface along U direction
    bsSurface.reverseU();
    std::cout << "Control Points after reversing U:" << std::endl;
    for (const auto& pt : bsSurface.controlPoints()) {
        std::cout << "(" << pt[0] << ", " << pt[1] << ", " << pt[2] << ")" << std::endl;
    }
    std::cout << std::endl;

    // Normalize the surface's basis along both U and V directions
    bsSurface.normalize();
    std::cout << "Surface evaluation after normalization at (u, v) = (0.5, 0.5):" << std::endl;
    Point3D normalizedSurfacePoint = bsSurface.eval(0.5, 0.5);
    std::cout << "(" << normalizedSurfacePoint[0] << ", " << normalizedSurfacePoint[1] << ", " << normalizedSurfacePoint[2] << ")" << std::endl << std::endl;

    // Insert knots to refine the surface
    BSSurface refinedSurface = bsSurface.insertKnotU(0.5, 2);
    refinedSurface = refinedSurface.insertKnotV(0.5, 2);
    std::cout << "Control Points after inserting knots in U and V:" << std::endl;
    for (const auto& pt : refinedSurface.controlPoints()) {
        std::cout << "(" << pt[0] << ", " << pt[1] << ", " << pt[2] << ")" << std::endl;
    }

    return 0;
}
