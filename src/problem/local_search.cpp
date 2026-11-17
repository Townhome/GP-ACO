#include "problem/local_search.h"
#include "core/random.h"
namespace gpaco {
std::pair<int, int> locateSegmentInRoute(TruckBase &pro, const TruckEncoding &ind, int truck,
                                         Real range1, Real range2, int min_pairs) {
    auto &routes = ind.m_truck_routes;
    auto &times = ind.m_arrive_time;

    int route_len = static_cast<int>(routes[truck].size());
    if (route_len < 2)
        return {-1, -1};

    int n_pairs = (route_len + 1) / 2; // number of pairs (int)

    // sanity
    if (n_pairs <= 0)
        return {-1, -1};

    Real shift = pro.getShiftTime();
    Real t1 = shift * range1;
    Real t2 = shift * range2;
    if (t1 > t2)
        std::swap(t1, t2);

    int left = -1;
    int right = -1;

    // find left
    for (int i = 0; i < n_pairs - 1; ++i) {
        Real start = times[truck][2 * i];
        Real end = times[truck][2 * (i + 1)];
        if (start >= t1) {
            left = i;
            break;
        } else if (start <= t1 && end >= t1) {
            Real dl = t1 - start;
            Real dr = end - t1;
            left = (dl <= dr) ? i : (i + 1);
            break;
        }
    }
    if (left == -1)
        return {-1, -1};

    // find right (start from left to avoid searching before left)
    for (int i = left; i < n_pairs - 1; ++i) {
        Real start = times[truck][2 * i];
        Real end = times[truck][2 * (i + 1)];
        if (start >= t2) {
            right = i;
            break;
        } else if (start <= t2 && end >= t2) {
            Real dl = t2 - start;
            Real dr = end - t2;
            right = (dl <= dr) ? i : (i + 1);
            break;
        }
    }
    if (right == -1)
        right = n_pairs - 1; // Not found, default to the last pair

    // Sanity clamp
    if (left < 0)
        left = 0;
    if (right < 0)
        right = 0;
    if (left >= n_pairs)
        left = n_pairs - 1;
    if (right >= n_pairs)
        right = n_pairs - 1;

    // If left > right, the positioning logic is abnormal - fix it to swap (or simply return
    // failure)
    if (left > right)
        std::swap(left, right);

    int len_pairs = right - left;
    if (len_pairs < min_pairs) {
        // Try expanding left first, then right
        int need = min_pairs - len_pairs;
        int can_left_extend = std::min(need, left); // left amount that can be expanded
        left -= can_left_extend;
        len_pairs = right - left;

        if (len_pairs < min_pairs) {
            need = min_pairs - len_pairs;
            int can_right_extend = std::min(need, (n_pairs - 1 - right));
            right += can_right_extend;
            len_pairs = right - left;
        }

        if (len_pairs < min_pairs) {
            // Unable to expand to a sufficient length, returns failure
            return {-1, -1};
        }
    }

    return {left, right};
}

void swapPairsBetweenRoutes(TruckBase &pro, TruckEncoding &ind, Real range1, Real range2,
                            int id_rnd) {
    auto &routes = ind.m_truck_routes;
    if (routes.size() < 2)
        return;

    size_t r1, r2;
    do {
        r1 = randomStream(id_rnd).uniform.nextNonStd<size_t>(0, routes.size());
    } while (routes[r1].size() < 2);
    do {
        r2 = randomStream(id_rnd).uniform.nextNonStd<size_t>(0, routes.size());
    } while (r2 == r1 || routes[r2].size() < 2);

    // Locate the pair of two trucks in [range1, range2]
    auto segA = locateSegmentInRoute(pro, ind, r1, range1, range2, 1); // (L1, R1)
    auto segB = locateSegmentInRoute(pro, ind, r2, range1, range2, 1); // (L2, R2)
    if (segA.first == -1 || segB.first == -1)
        return;

    int L1 = segA.first, R1 = segA.second;
    int L2 = segB.first, R2 = segB.second;
    if (L1 > R1 || L2 > R2)
        return;

    // Swap the segments
    std::vector<int> seg1(routes[r1].begin() + 2 * L1, routes[r1].begin() + 2 * R1);
    std::vector<int> seg2(routes[r2].begin() + 2 * L2, routes[r2].begin() + 2 * R2);
    routes[r1].erase(routes[r1].begin() + 2 * L1, routes[r1].begin() + 2 * R1);
    routes[r1].insert(routes[r1].begin() + 2 * L1, seg2.begin(), seg2.end());
    routes[r2].erase(routes[r2].begin() + 2 * L2, routes[r2].begin() + 2 * R2);
    routes[r2].insert(routes[r2].begin() + 2 * L2, seg1.begin(), seg1.end());
}

void reversePairsInRoute(TruckBase &pro, TruckEncoding &ind, Real range1, Real range2, int id_rnd) {
    auto &routes = ind.m_truck_routes;

    size_t r;
    do {
        r = randomStream(id_rnd).uniform.nextNonStd<size_t>(0, routes.size());
    } while (routes[r].size() < 4);

    auto seg = locateSegmentInRoute(pro, ind, r, range1, range2, 2); // (L, R)
    int L = seg.first, R = seg.second;
    int length = R - L;
    if (seg.first == -1 || L > R)
        return;

    for (size_t i = 0; i < length / 2; ++i) {
        size_t left = L + i;
        size_t right = R - 1 - i;
        std::swap(routes[r][left * 2], routes[r][right * 2]);         // swap load
        std::swap(routes[r][left * 2 + 1], routes[r][right * 2 + 1]); // swap unload
    }
}

void randomResetPairs(TruckBase &pro, TruckEncoding &ind, Real range1, Real range2, int id_rnd) {
    auto &routes = ind.m_truck_routes;
    int num_shovels = pro.getNumShovel();
    int num_unloads = pro.getNumUnload();

    size_t r;
    do {
        r = randomStream(id_rnd).uniform.nextNonStd<size_t>(0, routes.size());
    } while (routes[r].size() < 2);

    auto seg = locateSegmentInRoute(pro, ind, r, range1, range2, 1); // (L, R)
    int L = seg.first, R = seg.second;
    if (seg.first == -1 || L > R)
        return;

    // Randomly reset the segment
    for (size_t i = L; i < R; ++i) {
        size_t base = i * 2;
        // Reset load
        routes[r][base] = randomStream(id_rnd).uniform.nextNonStd<size_t>(0, num_shovels);
        // Reset unload
        routes[r][base + 1] =
            randomStream(id_rnd).uniform.nextNonStd<size_t>(num_shovels, num_shovels + num_unloads);
    }
}

void relocateSegmentInRoute(TruckBase &pro, TruckEncoding &ind, Real range1, Real range2,
                            int id_rnd) {
    auto &routes = ind.m_truck_routes;

    size_t r;
    do {
        r = randomStream(id_rnd).uniform.nextNonStd<size_t>(0, routes.size());
    } while (routes[r].size() < 2);

    auto seg = locateSegmentInRoute(pro, ind, r, range1, range2, 1); // (L, R)
    int L = seg.first, R = seg.second;
    if (seg.first == -1 || L > R)
        return;

    auto it1 = routes[r].begin() + L * 2;
    auto it2 = routes[r].begin() + R * 2;

    // Extract segment
    std::vector<int> segment(it1, it2);
    routes[r].erase(it1, it2);

    // Reinsert at random position
    size_t slots = (routes[r].size() + 1) / 2;
    size_t pos = randomStream(id_rnd).uniform.nextNonStd<size_t>(0, slots);
    routes[r].insert(routes[r].begin() + pos * 2, segment.begin(), segment.end());
}

void randomResetRange(TruckBase &pro, TruckEncoding &ind, Real range1, Real range2, int id_rnd) {
    auto &routes = ind.m_truck_routes;
    int num_shovels = pro.getNumShovel();
    int num_unloads = pro.getNumUnload();

    for (size_t r = 0; r < routes.size(); ++r) {
        if (routes[r].size() < 2)
            continue;

        auto seg = locateSegmentInRoute(pro, ind, r, range1, range2, 1); // (L, R)
        int L = seg.first, R = seg.second;
        if (seg.first == -1 || L > R)
            return;

        // Randomly reset the segment
        for (size_t i = L; i < R; ++i) {
            size_t base = i * 2;
            // Reset load
            routes[r][base] = randomStream(id_rnd).uniform.nextNonStd<size_t>(0, num_shovels);
            // Reset unload
            routes[r][base + 1] = randomStream(id_rnd).uniform.nextNonStd<size_t>(
                num_shovels, num_shovels + num_unloads);
        }
    }
}
} // namespace gpaco
