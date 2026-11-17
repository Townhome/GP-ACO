#pragma once
#include "core/types.h"
namespace gpaco {
class Uniform {
    double m_seed, m_buffer[128];
    double raw();

  public:
    explicit Uniform(double seed);
    double next();
    template <class T> T nextNonStd(T min, T max) {
        return min + static_cast<T>((max - min) * next());
    }
    template <class T> void shuffle(T first, T last) {
        for (auto i = last - first - 1; i > 0; --i) {
            auto t = decltype(i)(i + 1) * next();
            std::swap(first[i], first[t]);
        }
    }
};
struct RandomStream {
    Uniform uniform;
    explicit RandomStream(double seed) : uniform(seed) {}
};
int addRandom(double seed);
RandomStream &randomStream(int id);
void deleteRandom(int id);
void resetUniform(int id, uint64_t seed);
} // namespace gpaco
