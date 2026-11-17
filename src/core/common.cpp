#include "core/common.h"
#ifdef _WIN32
#include <windows.h>
#else
#include <fcntl.h>
#include <sys/file.h>
#include <unistd.h>
#endif
namespace gpaco {
FileLock::FileLock(const std::filesystem::path &path) {
#ifdef _WIN32
    HANDLE h =
        CreateFileW(path.c_str(), GENERIC_READ | GENERIC_WRITE, FILE_SHARE_READ | FILE_SHARE_WRITE,
                    nullptr, OPEN_ALWAYS, FILE_ATTRIBUTE_NORMAL, nullptr);
    require(h != INVALID_HANDLE_VALUE, "cannot open run lock");
    OVERLAPPED o{};
    if (!LockFileEx(h, LOCKFILE_EXCLUSIVE_LOCK | LOCKFILE_FAIL_IMMEDIATELY, 0, 1, 0, &o)) {
        CloseHandle(h);
        require(false, "another process owns " + path.string());
    }
    m_handle = reinterpret_cast<intptr_t>(h);
#else
    int h = ::open(path.c_str(), O_CREAT | O_RDWR, 0666);
    require(h >= 0, "cannot open run lock");
    if (::flock(h, LOCK_EX | LOCK_NB) != 0) {
        ::close(h);
        require(false, "another process owns " + path.string());
    }
    m_handle = h;
#endif
}
FileLock::~FileLock() {
    if (m_handle == -1)
        return;
#ifdef _WIN32
    HANDLE h = reinterpret_cast<HANDLE>(m_handle);
    OVERLAPPED o{};
    UnlockFileEx(h, 0, 1, 0, &o);
    CloseHandle(h);
#else
    ::flock(int(m_handle), LOCK_UN);
    ::close(int(m_handle));
#endif
}
std::string readText(const std::filesystem::path &path) {
    std::ifstream in(path, std::ios::binary);
    require(bool(in), "cannot open " + path.string());
    std::ostringstream out;
    out << in.rdbuf();
    require(!in.bad(), "cannot read " + path.string());
    return out.str();
}
uint64_t fileHash(const std::filesystem::path &path) {
    std::ifstream in(path, std::ios::binary);
    require(bool(in), "cannot open " + path.string());
    uint64_t h = 14695981039346656037ULL;
    char data[65536];
    while (in.read(data, sizeof(data)) || in.gcount())
        for (std::streamsize i = 0; i < in.gcount(); ++i)
            h = (h ^ static_cast<unsigned char>(data[i])) * 1099511628211ULL;
    require(in.eof(), "cannot hash " + path.string());
    return h;
}
void atomicWrite(const std::filesystem::path &path, const std::string &value) {
    if (!path.parent_path().empty())
        std::filesystem::create_directories(path.parent_path());
    auto tmp = path;
    tmp += ".tmp";
    std::ofstream out(tmp, std::ios::binary | std::ios::trunc);
    out << value;
    out.close();
    require(bool(out), "cannot write " + tmp.string());
#ifdef _WIN32
    require(MoveFileExW(tmp.c_str(), path.c_str(),
                        MOVEFILE_REPLACE_EXISTING | MOVEFILE_WRITE_THROUGH) != 0,
            "cannot commit " + path.string());
#else
    std::filesystem::rename(tmp, path);
#endif
}
} // namespace gpaco
