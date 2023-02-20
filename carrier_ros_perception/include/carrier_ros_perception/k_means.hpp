#ifndef KMEANS_HPP
#define KMEANS_HPP

#include <vector>

class KMeans {
public:
    KMeans(int num_clusters, int max_iterations = 100);
    std::vector<int> fit(const std::vector<float>& data);

private:
    int num_clusters_;
    int max_iterations_;
    std::vector<float> centroids_;
};
#endif // KMEANS_HPP
