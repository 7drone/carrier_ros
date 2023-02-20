#include "k_means.hpp"
#include <algorithm>
#include <random>

KMeans::KMeans(int num_clusters, int max_iterations)
    : num_clusters_(num_clusters), max_iterations_(max_iterations) {}

std::vector<int> KMeans::fit(const std::vector<float>& data) {
    std::vector<int> labels(data.size());
    std::vector<float> old_centroids(num_clusters_);
    centroids_.resize(num_clusters_);

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, data.size() - 1);

    // 랜덤으로 초기 centroid 선택
    for (int i = 0; i < num_clusters_; i++) {
        centroids_[i] = data[dis(gen)];
    }

    // max_iterations만큼 반복
    for (int iter = 0; iter < max_iterations_; iter++) {
        // 각 데이터 포인트에 대해 가장 가까운 centroid 찾기
        for (int i = 0; i < data.size(); i++) {
            int min_idx = 0;
            float min_dist = std::numeric_limits<float>::max();
            for (int j = 0; j < num_clusters_; j++) {
                float dist = std::abs(data[i] - centroids_[j]);
                if (dist < min_dist) {
                    min_dist = dist;
                    min_idx = j;
                }
            }
            labels[i] = min_idx;
        }

        // centroid 업데이트
        old_centroids = centroids_;
        for (int i = 0; i < num_clusters_; i++) {
            float sum = 0;
            int count = 0;
            for (int j = 0; j < data.size(); j++) {
                if (labels[j] == i) {
                    sum += data[j];
                    count++;
                }
            }
            if (count > 0) {
                centroids_[i] = sum / count;
            } else {
                centroids_[i] = data[dis(gen)];
            }
        }

        // centroid가 더이상 변하지 않으면 종료
        if (old_centroids == centroids_) {
            break;
        }
    }

    return labels;
}