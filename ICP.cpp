#include <Eigen/Dense>
#include <vector>
#include <memory>
#include "ICP.h"

ICP::ICP(const Config& config = Config()): config_(config) {};

Eigen::Matrix4d ICP::align(const PointCloud& source, const PointCloud& target) {
    PointCloud transformed = source;
    Eigen::Matrix4d transform = Matrix4d::Identity();

    for (int iter = 0; iter < config_.max_iters; iter++){
        auto correspondanced = findCorresponds(tansformed, target);
        auto [R, t] = computeTransform(corresponds, transformed, target);
        applyTransform(transformed, R, t);
        transform = updateTransform(R, t) * transform;

        if(checkConvergence(transformed, target)) break;
    }

    return transform;
}

std::vector<std::pair<size_t, size_t>> ICP::findCorresponds(const PointCloud& source, const PointCloud& target) {
    std::vector<std::pair<size_t, size_t>> corresponds;

    for (size_t i = 0; i < source.size(); i++) {
        double min_dist = std::numeric_limits<double>::max();
        size_t closest_idx = 0;
        for (size_t j = 0; j < target.size(); j++) {
            double dist = (source[i].coords - target[j].coords).norm();
            if(dist < min_dist){
                min_dist = dist;
                closest_idx = j;
            }
        }
        if (min_dist < config_.max_distance) {
            corresponds.emplace_back(i, closest_idx);
        }
    }

    return corresponds;
}

// COmpute the Transformation Matrix using SVD
std::pair<Eigen::Matrix4d, Eigen::Vector3d> ICP::computeTransform (
    const std::vector<std::pair<size_t, size_t>>& corresponds,
    const PointCloud& source,
    const PointCloud& target
) {
    Eigen::Vector3d src_centroid = computeCentroid(source, corresponds);
    Eigen::Vector3d tgt_centroid = computeCentroid(target, corresponds);

    Eigen::Matrix3d H = Eigen::Matrix3d::Zero();
    
    for (const auto& [src_idx, tgt_idx] : corresponds) {
        H += (source[src_idx].coords - src_centroid) * (target[tgt_idx].coords - tgt_centroid).transpose();
    }

    Eigen::JacobiSVD<Eigen::Matrix3d> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d R = svd.matrixU() * svd.matrixV().transpose();

    if (R.determinant() < 0) R.cols(2) *= -1;
    Eigen::Vector3d t = tgt_centroid - R * src_centroid;

    return {R, t};

}

void ICP::applyTransform(PointCloud& cloud, const Eigen::Matrix3d& R, const Eigen::Vector3d& t) {
    for (auto& p : cloud.points) {
        p.coords = R * p.coords + t;
    }
}

Eigen::Matrix4d ICP::updateTransform(const Eigen::Matrix3d& R, const Eigen::Vector3d& t) {
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3, 3>(0, 0) = R;
    T.block<3, 1>(0, 3) = t;

    return T;
}

Eigen::Vector3d ICP::computeCentroid(const PointCloud& cloud, 
    const std::vector<std::pair<size_t, size_t>>& corresponds,
    bool use_source) {

    Eigen::Vector3d centroid = Eigen:Vector3d::Zero();
    size_t count = 0;

    for (const auto& [src_idx, tgt_idx] : corresponds) {
        centroid += use_source ? cloud[src_idx].coords : cloud[tgt_idx].coords;
        count++;
    }

    if (count > 0) centroid /= count;
    return centroid;
}


bool ICP::checkConvergence(const PointCloud& target, const PointCloud& transformed) {
    // Apply RMSE
    double rmse = 0.0;
    size_t count = 0;

    for (size_t i = 0; i < transformed.size(); i++) {
        double dist = (transformed[i].coords - target[i].coords).norm();
        rmse += dist * dist;
        count++;
    }

    if (count > 0) rmse = std::sqrt(rmse / count);
    return rmse < config_.convergence_threshold;

}