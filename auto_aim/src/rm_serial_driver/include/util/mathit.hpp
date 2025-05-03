#include <cmath>
#include <numeric>
#include <vector>
#include <array>

template<typename Container>
auto norm(const Container& vec) -> decltype(std::sqrt(std::declval<typename Container::value_type>())) {
    using T = typename Container::value_type;
    T sum_sq = std::accumulate(vec.begin(), vec.end(), T(0), [](T acc, T val) {
        return acc + val * val;
    });
    return std::sqrt(sum_sq);
}
