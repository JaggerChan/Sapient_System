#include "sapient_nodeid.h"
#include <iostream>
#include <vector>
#include <cstring>
#include <cstdint>
#include <sstream>
#include <iomanip>
#include <random>
#include <fstream>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <errno.h>
#include <string>

// 引用外部定义的序列号全局变量
extern std::string g_sn;

// 持久化 Node ID 文件路径
static const char* NODE_ID_FILE_PATH = "/home/chenyl/node_id.txt";

/* SHA-1 实现 - 用于 UUID v5 生成 */
class SHA1 {
private:
    uint32_t h[5];
    uint8_t buffer[64];
    size_t buffer_len;
    uint64_t total_len;
    
    static inline uint32_t left_rotate(uint32_t value, size_t count) {
        return (value << count) | (value >> (32 - count));
    }
    
    void process_chunk(const uint8_t* chunk) {
        uint32_t w[80];
        
        // 扩展 16 个 32 位字到 80 个
        for (int i = 0; i < 16; i++) {
            w[i] = (chunk[i * 4] << 24) | (chunk[i * 4 + 1] << 16) | 
                   (chunk[i * 4 + 2] << 8) | chunk[i * 4 + 3];
        }
        for (int i = 16; i < 80; i++) {
            w[i] = left_rotate(w[i - 3] ^ w[i - 8] ^ w[i - 14] ^ w[i - 16], 1);
        }
        
        uint32_t a = h[0], b = h[1], c = h[2], d = h[3], e = h[4];
        
        for (int i = 0; i < 80; i++) {
            uint32_t f, k;
            if (i < 20) {
                f = (b & c) | ((~b) & d);
                k = 0x5A827999;
            } else if (i < 40) {
                f = b ^ c ^ d;
                k = 0x6ED9EBA1;
            } else if (i < 60) {
                f = (b & c) | (b & d) | (c & d);
                k = 0x8F1BBCDC;
            } else {
                f = b ^ c ^ d;
                k = 0xCA62C1D6;
            }
            
            uint32_t temp = left_rotate(a, 5) + f + e + k + w[i];
            e = d;
            d = c;
            c = left_rotate(b, 30);
            b = a;
            a = temp;
        }
        
        h[0] += a; h[1] += b; h[2] += c; h[3] += d; h[4] += e;
    }
    
public:
    SHA1() {
        reset();
    }
    
    void reset() {
        h[0] = 0x67452301;
        h[1] = 0xEFCDAB89;
        h[2] = 0x98BADCFE;
        h[3] = 0x10325476;
        h[4] = 0xC3D2E1F0;
        buffer_len = 0;
        total_len = 0;
    }
    
    void update(const uint8_t* data, size_t len) {
        total_len += len;
        
        for (size_t i = 0; i < len; i++) {
            buffer[buffer_len++] = data[i];
            if (buffer_len == 64) {
                process_chunk(buffer);
                buffer_len = 0;
            }
        }
    }
    
    void finalize(uint8_t* digest) {
        buffer[buffer_len++] = 0x80;
        
        if (buffer_len > 56) {
            while (buffer_len < 64) buffer[buffer_len++] = 0;
            process_chunk(buffer);
            buffer_len = 0;
        }
        
        while (buffer_len < 56) buffer[buffer_len++] = 0;
        
        // 添加长度（以位为单位，大端序）
        uint64_t bit_len = total_len * 8;
        for (int i = 7; i >= 0; i--) {
            buffer[56 + i] = (bit_len >> (i * 8)) & 0xFF;
        }
        
        process_chunk(buffer);
        
        // 输出哈希值（大端序）
        for (int i = 0; i < 5; i++) {
            digest[i * 4] = (h[i] >> 24) & 0xFF;
            digest[i * 4 + 1] = (h[i] >> 16) & 0xFF;
            digest[i * 4 + 2] = (h[i] >> 8) & 0xFF;
            digest[i * 4 + 3] = h[i] & 0xFF;
        }
    }
};

/* 将 UUID 字符串转换为 16 字节二进制格式 */
static void uuid_string_to_bytes(const std::string& uuid_str, uint8_t* uuid_bytes) {
    // 移除连字符
    std::string clean;
    for (size_t i = 0; i < uuid_str.length(); i++) {
        if (uuid_str[i] != '-') {
            clean += uuid_str[i];
        }
    }
    
    // 转换为字节
    for (int i = 0; i < 16; i++) {
        std::string byte_str = clean.substr(i * 2, 2);
        uuid_bytes[i] = static_cast<uint8_t>(std::stoul(byte_str, nullptr, 16));
    }
}

/* 将 16 字节二进制格式转换为 UUID 字符串 */
static std::string uuid_bytes_to_string(const uint8_t* uuid_bytes) {
    std::ostringstream oss;
    oss << std::hex << std::setfill('0');
    
    for (int i = 0; i < 16; i++) {
        if (i == 4 || i == 6 || i == 8 || i == 10) {
            oss << '-';
        }
        oss << std::setw(2) << static_cast<unsigned>(uuid_bytes[i]);
    }
    
    return oss.str();
}

/* ===== UUID v5 生成函数（长期方案保留） ===== */

/* 基于设备序列号生成 UUID v5 */
static std::string generateUUIDv5(const std::string& serial_number) 
{
    // RFC 4122 标准定义的 DNS 命名空间 UUID (6ba7b810-9dad-11d1-80b4-00c04fd430c8)
    const std::string NAMESPACE_DNS = "6ba7b810-9dad-11d1-80b4-00c04fd430c8";
    uint8_t namespace_bytes[16];
    uuid_string_to_bytes(NAMESPACE_DNS, namespace_bytes);
    
    // 准备输入数据：命名空间 UUID (16字节) + "SDH100" + 设备序列号
    std::vector<uint8_t> input_data;
    input_data.insert(input_data.end(), namespace_bytes, namespace_bytes + 16);
    std::string name = "SDH100" + serial_number;
    input_data.insert(input_data.end(), name.begin(), name.end());
    
    // 计算 SHA-1 哈希
    SHA1 sha1;
    sha1.reset();
    sha1.update(input_data.data(), input_data.size());
    
    uint8_t hash[20];
    sha1.finalize(hash);
    
    // 从 SHA-1 哈希的前 16 字节创建 UUID
    uint8_t uuid_bytes[16];
    std::memcpy(uuid_bytes, hash, 16);
    
    // 设置 UUID 版本为 5（第 6 字节的高 4 位）
    uuid_bytes[6] = (uuid_bytes[6] & 0x0F) | 0x50;
    
    // 设置变体位为 10（第 8 字节的高 2 位）
    uuid_bytes[8] = (uuid_bytes[8] & 0x3F) | 0x80;
    
    // 转换为 UUID 字符串格式
    return uuid_bytes_to_string(uuid_bytes);
}

/* 从持久化文件读取 Node ID */
static std::string readNodeIDFromFile() {
    std::ifstream file(NODE_ID_FILE_PATH);
    if (!file.is_open()) {
        return "";  // 文件不存在或无法打开
    }
    
    std::string node_id;
    std::getline(file, node_id);
    file.close();
    
    // 简单验证：UUID 应该是 36 个字符（8-4-4-4-12 格式）
    if (node_id.length() == 36 && node_id[8] == '-' && node_id[13] == '-' && 
        node_id[18] == '-' && node_id[23] == '-') {
        std::cout << "Node ID loaded from file: " << node_id << std::endl;
        return node_id;
    }
    
    std::cerr << "Warning: Invalid Node ID format in file, will regenerate" << std::endl;
    return "";
}

/* 确保目录存在（如果不存在则创建） */
static bool ensureDirectoryExists(const char* dir_path) {
    struct stat info;
    if (stat(dir_path, &info) == 0) {
        // 目录已存在
        if (S_ISDIR(info.st_mode)) {
            return true;
        } else {
            std::cerr << "Error: " << dir_path << " exists but is not a directory" << std::endl;
            return false;
        }
    }
    
    // 目录不存在，尝试创建
    // 注意：这里只创建单层目录，如果 /home 不存在会失败
    if (mkdir(dir_path, S_IRWXU | S_IRGRP | S_IXGRP | S_IROTH | S_IXOTH) == 0) {
        std::cout << "Created directory: " << dir_path << std::endl;
        return true;
    } else {
        std::cerr << "Error: Failed to create directory " << dir_path << ": " << strerror(errno) << std::endl;
        return false;
    }
}

/* 将 Node ID 持久化到文件 */
static bool writeNodeIDToFile(const std::string& node_id) {
    // 提取目录路径（/home/chenyl）
    std::string file_path(NODE_ID_FILE_PATH);
    size_t last_slash = file_path.find_last_of('/');
    if (last_slash == std::string::npos) {
        std::cerr << "Error: Invalid file path: " << NODE_ID_FILE_PATH << std::endl;
        return false;
    }
    
    std::string dir_path = file_path.substr(0, last_slash);
    
    // 确保目录存在
    if (!ensureDirectoryExists(dir_path.c_str())) {
        std::cerr << "Error: Cannot ensure directory exists: " << dir_path << std::endl;
        return false;
    }
    
    // 打开文件进行写入
    // std::ios::out | std::ios::trunc 表示：
    // - out: 输出模式（写入）
    // - trunc: 如果文件存在则截断（清空），如果不存在则创建
    std::ofstream file(NODE_ID_FILE_PATH, std::ios::out | std::ios::trunc);
    if (!file.is_open()) {
        std::cerr << "Error: Failed to open " << NODE_ID_FILE_PATH << " for writing: " << strerror(errno) << std::endl;
        return false;
    }
    
    // 写入 Node ID
    file << node_id << std::endl;
    file.close();
    
    // 验证文件是否成功写入
    if (!file.good()) {
        std::cerr << "Error: Failed to write Node ID to file" << std::endl;
        return false;
    }
    
    // 设置文件权限为 644（rw-r--r--）
    if (chmod(NODE_ID_FILE_PATH, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH) != 0) {
        std::cerr << "Warning: Failed to set file permissions: " << strerror(errno) << std::endl;
        // 权限设置失败不影响文件创建，继续执行
    }
    
    std::cout << "Node ID saved to file: " << node_id << std::endl;
    return true;
}

/* ===== UUID v4 生成函数（短期方案使用） ===== */

/* 生成随机的 UUID v4 */
static std::string generateUUIDv4() 
{
    uint8_t uuid_bytes[16];
    
    // 初始化随机数生成器
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<unsigned short> dis(0, 255);
    
    // 生成随机字节
    for (int i = 0; i < 16; ++i) {
        uuid_bytes[i] = static_cast<uint8_t>(dis(gen));
    }
    
    // 设置版本位为 4 (UUID v4) - 第 6 字节的高 4 位
    uuid_bytes[6] = (uuid_bytes[6] & 0x0F) | 0x40;
    
    // 设置变体位 (Variant) 为 10xx - 第 8 字节的高 2 位
    uuid_bytes[8] = (uuid_bytes[8] & 0x3F) | 0x80;
    
    return uuid_bytes_to_string(uuid_bytes);
}

/* ===== 短期方案：强制使用 UUID v4（兼容客户系统） ===== */

/*
 * 生成 NodeID - 短期方案（当前）
 * 
 * 策略：文件 > UUID v4（持久化）
 * - 优先从文件读取（已有设备保持不变）
 * - 文件不存在时，**始终生成 UUID v4**（不管是否有序列号）
 * - 持久化到文件，确保重启后不变
 * 
 * 理由：客户系统只支持 UUID v4，必须强制使用 v4
 */
std::string generateNodeID() 
{
    // 内存缓存，避免重复读取文件
    static std::string cached_node_id;
    static bool cache_initialized = false;
    
    // 如果已经缓存，直接返回
    if (cache_initialized && !cached_node_id.empty()) {
        return cached_node_id;
    }
    
    // 第一步：尝试从持久化文件读取（最高优先级）
    std::string node_id = readNodeIDFromFile();
    if (!node_id.empty()) {
        cached_node_id = node_id;
        cache_initialized = true;
        std::cout << "[NodeID] Using persistent Node ID from file: " << node_id << std::endl;
        return cached_node_id;
    }
    
    // 第二步：文件不存在，强制生成 UUID v4（短期方案）
    // 注意：不管是否有序列号，都使用 UUID v4，确保客户系统兼容性
    std::cout << "[NodeID] No persistent file found, generating new UUID v4 for customer compatibility" << std::endl;
    if (!g_sn.empty()) {
        std::cout << "[NodeID] Device SN available: " << g_sn << " (but using v4 for compatibility)" << std::endl;
    } else {
        std::cout << "[NodeID] Device SN not available" << std::endl;
    }
    
    node_id = generateUUIDv4();  // 短期方案：强制 UUID v4
    
    // 第三步：持久化保存到文件
    if (writeNodeIDToFile(node_id)) {
        std::cout << "[NodeID] UUID v4 successfully persisted to file: " << node_id << std::endl;
    } else {
        std::cerr << "[NodeID] Warning: Failed to persist Node ID to file, will use in-memory only" << std::endl;
    }
    
    // 缓存到内存
    cached_node_id = node_id;
    cache_initialized = true;
    
    return cached_node_id;
}

/* ===== 长期方案接口（预留，未来实现） ===== */

/*
 * 根据配置生成 NodeID - 长期方案（未来）
 * 
 * @param use_uuid_v5 true: 使用 UUID v5（基于序列号）, false: 使用 UUID v4（随机）
 * @return Node ID 字符串
 * 
 * 使用场景：
 * - 从配置文件或 Web UI 读取用户选择的 UUID 版本
 * - 调用对应的生成函数
 * 
 * 示例：
 *   bool use_v5 = config["sapient"]["uuid_version"] == "v5";
 *   std::string node_id = generateNodeIDWithConfig(use_v5);
 */
std::string generateNodeIDWithConfig(bool use_uuid_v5) 
{
    // 第一步：优先从文件读取（已持久化）
    static std::string cached_node_id;
    static bool cache_initialized = false;
    
    if (cache_initialized && !cached_node_id.empty()) {
        return cached_node_id;
    }
    
    std::string node_id = readNodeIDFromFile();
    if (!node_id.empty()) {
        cached_node_id = node_id;
        cache_initialized = true;
        std::cout << "[NodeID] Using persistent Node ID from file" << std::endl;
        return cached_node_id;
    }
    
    // 第二步：根据配置生成（长期方案）
    if (use_uuid_v5 && !g_sn.empty()) {
        std::cout << "[NodeID] Config: UUID v5, generating based on SN: " << g_sn << std::endl;
        node_id = generateUUIDv5(g_sn);
    } else {
        if (use_uuid_v5 && g_sn.empty()) {
            std::cerr << "[NodeID] Warning: Config requires v5 but SN not available, falling back to v4" << std::endl;
        } else {
            std::cout << "[NodeID] Config: UUID v4, generating random UUID" << std::endl;
        }
        node_id = generateUUIDv4();
    }
    
    // 第三步：持久化
    writeNodeIDToFile(node_id);
    cached_node_id = node_id;
    cache_initialized = true;
    
    return cached_node_id;
}

