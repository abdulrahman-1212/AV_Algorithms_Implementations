#include "RANSAC.h"
#include <Eigen/Eigen>





RANSAC::RANSAC(const Config& config) : config_(config) {};

std::pair<Plane, std::vector<size_t>> RANSAC::fitPlane(const PointCloud& cloud) {
    std::vector<size_t> best_inliers;
    Plane best_plane;

    std::random_device rd;
    std::mt19937 gen(rd());

    for (int i = 0; i < config_.max_iters; i++) {
        std::vector<size_t> samplePoints = randomSample(cloud.size(), 3, gen);
        Plane plane = fitPlaneFromPoints(cloud, samplePoints);

        std::vector<size_t> inliers = getInliers(cloud, plane);

        if (inliers.size() > best_inliers.size()){
            best_inliers = inliers;
            best_plane = plane;
        }

        if (best_inliers.size() >= config_.min_inliers) break;
    }

    best_plane = refinePlane(cloud, best_inliers);
    return {best_plane, best_inliers};
    
}


std::vector<size_t> RANSAC::randomSample(size_t n, size_t k, std::mt19937& gen) {
    
    std::vector<size_t> indices(n);
    std::iota(indices.begin(), indices.end(), 0);
    std::shuffle(indices.begin(), indices.end(), gen);
    // return the first k points after shuffling
    return std::vector<size_t> (indices.begin(), indices.begin() + k);
};


Plane RANSAC::fitPlaneFromPoints(const PointCloud& cloud, std::vector<size_t>& indices) {
    Eigen::Vector3d p1 = cloud[indices[0]].coords;
    Eigen::Vector3d p2 = cloud[indices[1]].coords;
    Eigen::Vector3d p2 = cloud[indices[2]].coords;

    Eigen::Vector3d normal = (p2 - p1).cross(p3 - p1).normalizded()         // vector normal to the plane
    double d = -normal.dot(p1);
    return {normal, d};
};

std::vector<size_t> RANSAC::getInliers(const PointCloud& cloud, const Plane& plane) {
    std::vector<size_t> inliers;
    for (size_t i = 0; i < cloud.size(); i++) {
        double dist = std::abs(plane.normal.dot(cloud[i].coords) + plane.d);
        if (dist < config_.dist_threshold) {
            inliers.push_back(i);
        }
    } 

    return inliers;
};


Plane RANSAC::refinePlane(const PointCloud& cloud, std::vector<size_t>& inliers) {
    Eigen::MatrixXd A(inliers.size(), 3);
    Eigen::VectorXd b(inliers.size());

    for (size_t i = 0; i < inliers.size(); i++) {
        A.row(i) = cloud[inliers[i]].coords;
        b[i] = -1.0;
    }

    Eigen::Vector3d normal = A.colPivHouseholderQr().solve(b).normalized();
    double d = -normal.dot(A.row(0));
    return {normal, d};
};
