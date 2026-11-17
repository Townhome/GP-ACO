#pragma once
#include "problem/simulator.h"
namespace gpaco {
std::pair<int, int> locateSegmentInRoute(TruckBase &pro, const TruckEncoding &ind, int truck,
                                         Real range1, Real range2, int min_pairs);
void swapPairsBetweenRoutes(TruckBase &pro, TruckEncoding &ind, Real range1, Real range2,
                            int id_rnd);
void reversePairsInRoute(TruckBase &pro, TruckEncoding &ind, Real range1, Real range2, int id_rnd);
void randomResetPairs(TruckBase &pro, TruckEncoding &ind, Real range1, Real range2, int id_rnd);
void relocateSegmentInRoute(TruckBase &pro, TruckEncoding &ind, Real range1, Real range2,
                            int id_rnd);
void randomResetRange(TruckBase &pro, TruckEncoding &ind, Real range1, Real range2, int id_rnd);
} // namespace gpaco
