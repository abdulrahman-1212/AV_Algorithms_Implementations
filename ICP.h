#include <Eigen/Eigen>
#include <vector>
#ifndef ICP_H
#define ICP_H

typedef struct Point3D {
    Eigen::Vector3d Coords;
    Point3D(double x, double y, double z) : coords(x, y, z) {}
} Point3D;

class PointCloud {
    public:
        std::vector<Point3D> points;
        void addPoint(const Point3D& p) { point.push_back(p); }
        size_t size() const { return points.size(); }
        // Overloading Index Operator
        Point3D& operator[](size_t i) { return points[i]; }
        const Point3D& operator[](size_t i) const { return points[i]; }

};

class ICP {

public:
    // ICP configuration parameters
    struct Config {
        double max_distance = 1.0;
        double max_iters = 50;
        double convergence_threshold = 1e-6;
    };
    ICP(const Config& config = Config()): config_(config) {};

    Eigen::Matrix4d align(const PointCloud&, const PointCloud&);

private:
    Config config_;

    std::vector<std::pair<size_t, size_t>> findCorrespondances(const PointCloud&, const PointCloud&);

    std::pair<Eigen::Matrix4d, Eigen::Vector3d> computeTransform (
        const std::vector<std::pair<size_t, size_t>>&,
        const PointCloud&,
        const PointCloud&
    );

    void applyTransform(PointCloud&, const Eigen::Matrix3d&, const Eigen::Vector3d&);

    Eigen::Matrix4d updateTransform(const Eigen::Matrix3d&, const Eigen::Vector3d&);

    Eigen::Vector3d computeCentroid(const PointCloud&, const std::vector<std::pair<size_t, size_t>>&, bool);

    bool checkConvergence(const PointCloud&, const PointCloud&);



}

#endif