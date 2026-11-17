#include "core/random.h"
#include <new>
#include <stdexcept>
namespace gpaco {
double Uniform::raw() // get new uniform random number
{
    // m = 2147483647 = 2^31 - 1; a = 16807;
    // 127773 = m div a; 2836 = m mod a
    long iseed = (long)m_seed;
    long hi = iseed / 127773L;      // integer division
    long lo = iseed - hi * 127773L; // modulo
    iseed = 16807 * lo - 2836 * hi;
    if (iseed <= 0)
        iseed += 2147483647L;
    m_seed = (double)iseed;
    return m_seed * 4.656612875e-10;
}
double Uniform::next() // get new mixed random number
{
    if (!m_seed)
        throw std::runtime_error("RandBase number generator not initialised@Uniform::next");

    int i = (int)(raw() * 128); // 0 <= i < 128

    double f = m_buffer[i];
    m_buffer[i] = raw(); // Microsoft release gets this wrong
    return f;

    // return Mother(&iseed);
}
Uniform::Uniform(double s) // set random number seed
                           // s must be between 0 and 1
{

    if (s >= 1.0 || s <= 0.0)
        throw std::runtime_error("Newran: seed out of range@Uniform::Uniform");

    // iseed = 2147483648L * s;         // for Mother

    m_seed = (long)(s * 2147483648L);
    for (int i = 0; i < 128; i++)
        m_buffer[i] = raw();
}
namespace {
std::vector<std::unique_ptr<RandomStream>> streams;
}
int addRandom(double seed) {
    for (size_t i = 0; i < streams.size(); ++i)
        if (!streams[i]) {
            streams[i] = std::make_unique<RandomStream>(seed);
            return int(i);
        }
    streams.push_back(std::make_unique<RandomStream>(seed));
    return int(streams.size() - 1);
}
RandomStream &randomStream(int id) {
    return *streams.at(id);
}
void deleteRandom(int id) {
    streams.at(id).reset();
}
void resetUniform(int id, uint64_t seed) {
    auto &uniform = randomStream(id).uniform;
    uniform.~Uniform();
    const double normalized =
        std::clamp((double(seed >> 11) + .5) / 9007199254740992.0,
                   std::numeric_limits<double>::epsilon(), std::nextafter(1., 0.));
    new (&uniform) Uniform(normalized);
}
} // namespace gpaco
