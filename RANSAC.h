#include <vector>
#include <Eigen/Eigen>

#ifndef RANSAC_H
#define RANSAC_H

struct Point3d {
    Eigen::Vector3d coords;
    Point3d(double x, double y, double z) : coords(x, y, z) {};
};

struct PointCloud {
    std::vector<Point3d> points;
    void addPoint(const Point3d& p) { points.push_back(po); }
    size_t size() const { return points.size(); }
    Point3d& operator[](size_t idx) { return points[idx]; }
    const Point3D& operator[](size_t i) const { return points[i]; }

}


class RANSAC {
public:
    struct Config {
        int max_iters = 1000;
        double dist_threshold = 0.01;
        int min_inliers = 10;

    };

    struct Plane {
        Eigen::Vector3d normal;
        int distance;
    };

    RANSAC(const Config&);
    std::pair<Plane, std::vector<size_t>> fitPlane(const PointCloud& cloud);

private:
    Config config_;

    std::vector<size_t> randomSample(size_t n, size_t k, std::mt19937& gen);        // std::mt19937& reference to Mersenne Twister random number generator

    Plane fitPlaneFromPoints(const PointCloud& cloud, std::vector<size_t>& indices);

    std::vector<size_t> getInliers(const PointCloud& cloud, const Plane& plane);

    Plane refinePlane(const PointCloud& cloud, std::vector<size_t>& inliers)

}

#endif
