#include <cstdint>
#include <iostream>

#include "generated_tflite_classifier_artifact.hpp"

int main() {
    int coordinate0 = 0;
    int coordinate1 = 0;
    int coordinate2 = 0;
    int coordinate3 = 0;
    while (std::cin >> coordinate0 >> coordinate1 >> coordinate2 >> coordinate3) {
        const std::int8_t feature[4] = {
            static_cast<std::int8_t>(coordinate0),
            static_cast<std::int8_t>(coordinate1),
            static_cast<std::int8_t>(coordinate2),
            static_cast<std::int8_t>(coordinate3),
        };
        const auto score = ls2k::vision::ml::generated::ScoreIdentityFeature(feature);
        std::cout << score.class_id;
        for (const int distance : score.class_distances) std::cout << ' ' << distance;
        std::cout << ' ' << score.margin << '\n';
    }
    return std::cin.eof() ? 0 : 2;
}
