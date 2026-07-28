#include "port/platform_adapter.hpp"

// 参数存储实现 —— 从 JSON 配置文件中加载当前运行时参数。
// 支持 JSON 注释扩展、严格语法校验、文件读取和 OpenCV FileStorage 字段提取。
// 文件、语法或字段校验失败时拒绝加载；合法可选字段缺失仍保留字段默认值。

#include <array>
#include <cstddef>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <limits>
#include <sstream>
#include <string>
#include <unordered_set>
#include <vector>

#include <opencv2/core/persistence.hpp>

#include "control/wheel_target_mixer.hpp"
#include "control/steering_yaw_controller.hpp"
#include "port/actuator_command_types.hpp"
#include "port/runtime_parameter_validation.hpp"

namespace ls2k::platform {
namespace {

/**
 * 读取文件全部内容到字符串。
 * @param path 文件路径
 * @param out 输出参数，接收文件内容字符串
 * @return true 表示读取成功，false 表示文件无法打开
 */
bool ReadText(const std::string& path, std::string& out) {
    std::ifstream input(path);
    if (!input.is_open()) {
        return false;
    }
    std::stringstream buffer;
    buffer << input.rdbuf();
    out = buffer.str();
    return true;
}

// 在严格 JSON 校验和 OpenCV 字段提取前剥离配置支持的注释扩展。
// 注释被替换为空白，避免把注释两侧的 token 拼接成另一个合法 token。
// 未闭合块注释返回 false；字符串闭合由后续严格 JSON 校验器判定。
bool StripJsonComments(const std::string& text, std::string& output) {
    output.clear();
    output.reserve(text.size());

    bool in_string = false;
    bool escaped = false;
    bool in_line_comment = false;
    bool in_block_comment = false;

    for (std::size_t i = 0; i < text.size(); ++i) {
        const char c = text[i];
        const char next = (i + 1 < text.size()) ? text[i + 1] : '\0';

        if (in_line_comment) {
            if (c == '\n') {
                in_line_comment = false;
                output.push_back(c);
            } else {
                output.push_back(' ');
            }
            continue;
        }

        if (in_block_comment) {
            if (c == '\n') {
                output.push_back(c);
                continue;
            }
            if (c == '*' && next == '/') {
                in_block_comment = false;
                output.push_back(' ');
                output.push_back(' ');
                ++i;
            } else {
                output.push_back(' ');
            }
            continue;
        }

        if (in_string) {
            output.push_back(c);
            if (escaped) {
                escaped = false;
            } else if (c == '\\') {
                escaped = true;
            } else if (c == '"') {
                in_string = false;
            }
            continue;
        }

        if (c == '"' ) {
            in_string = true;
            output.push_back(c);
            continue;
        }

        if (c == '/' && next == '/') {
            in_line_comment = true;
            output.push_back(' ');
            output.push_back(' ');
            ++i;
            continue;
        }

        if (c == '/' && next == '*') {
            in_block_comment = true;
            output.push_back(' ');
            output.push_back(' ');
            ++i;
            continue;
        }

        output.push_back(c);
    }

    return !in_block_comment;
}

class StrictJsonObjectSyntax final {
public:
    explicit StrictJsonObjectSyntax(const std::string& text) : text_(text) {}

    bool ParseDocument(std::string& canonicalized) {
        SkipWhitespace();
        if (!ParseObject()) {
            return false;
        }
        SkipWhitespace();
        if (position_ != text_.size()) {
            return false;
        }
        BuildCanonicalizedDocument(canonicalized);
        return true;
    }

private:
    struct MemberNameReplacement {
        std::size_t begin = 0;
        std::size_t end = 0;
        std::string decoded{};
    };

    bool ParseValue() {
        SkipWhitespace();
        if (position_ >= text_.size()) {
            return false;
        }
        switch (text_[position_]) {
            case '{': return ParseObject();
            case '[': return ParseArray();
            case '"': return ParseString(nullptr);
            case 't': return ConsumeLiteral("true");
            case 'f': return ConsumeLiteral("false");
            case 'n': return ConsumeLiteral("null");
            default: return ParseNumber();
        }
    }

    bool ParseObject() {
        if (!Consume('{')) {
            return false;
        }
        SkipWhitespace();
        if (Consume('}')) {
            return true;
        }
        std::unordered_set<std::string> member_names;
        while (true) {
            SkipWhitespace();
            const std::size_t member_name_begin = position_;
            std::string member_name;
            if (!ParseString(&member_name) || !member_names.insert(member_name).second) {
                return false;
            }
            member_name_replacements_.push_back(
                {member_name_begin, position_, member_name});
            SkipWhitespace();
            if (!Consume(':') || !ParseValue()) {
                return false;
            }
            SkipWhitespace();
            if (Consume('}')) {
                return true;
            }
            if (!Consume(',')) {
                return false;
            }
        }
    }

    bool ParseArray() {
        if (!Consume('[')) {
            return false;
        }
        SkipWhitespace();
        if (Consume(']')) {
            return true;
        }
        while (true) {
            if (!ParseValue()) {
                return false;
            }
            SkipWhitespace();
            if (Consume(']')) {
                return true;
            }
            if (!Consume(',')) {
                return false;
            }
        }
    }

    bool ParseString(std::string* decoded) {
        if (!Consume('"')) {
            return false;
        }
        while (position_ < text_.size()) {
            const unsigned char c = static_cast<unsigned char>(text_[position_++]);
            if (c == '"') {
                return true;
            }
            if (c < 0x20) {
                return false;
            }
            if (c != '\\') {
                if (decoded != nullptr) {
                    decoded->push_back(static_cast<char>(c));
                }
                continue;
            }
            if (position_ >= text_.size()) {
                return false;
            }
            const char escape = text_[position_++];
            if (escape == '"' || escape == '\\' || escape == '/') {
                if (decoded != nullptr) {
                    decoded->push_back(escape);
                }
                continue;
            }
            if (escape == 'b' || escape == 'f' || escape == 'n' || escape == 'r' || escape == 't') {
                if (decoded != nullptr) {
                    switch (escape) {
                        case 'b': decoded->push_back('\b'); break;
                        case 'f': decoded->push_back('\f'); break;
                        case 'n': decoded->push_back('\n'); break;
                        case 'r': decoded->push_back('\r'); break;
                        case 't': decoded->push_back('\t'); break;
                    }
                }
                continue;
            }
            if (escape != 'u') {
                return false;
            }
            unsigned int code_point = 0;
            if (!ParseHexCodeUnit(code_point)) {
                return false;
            }
            if (code_point >= 0xD800 && code_point <= 0xDBFF) {
                if (position_ + 2 > text_.size() || text_[position_] != '\\' ||
                    text_[position_ + 1] != 'u') {
                    return false;
                }
                position_ += 2;
                unsigned int low_surrogate = 0;
                if (!ParseHexCodeUnit(low_surrogate) ||
                    low_surrogate < 0xDC00 || low_surrogate > 0xDFFF) {
                    return false;
                }
                code_point = 0x10000 + ((code_point - 0xD800) << 10) +
                             (low_surrogate - 0xDC00);
            } else if (code_point >= 0xDC00 && code_point <= 0xDFFF) {
                return false;
            }
            if (decoded != nullptr) {
                AppendUtf8(code_point, *decoded);
            }
        }
        return false;
    }

    bool ParseHexCodeUnit(unsigned int& value) {
        if (position_ + 4 > text_.size()) {
            return false;
        }
        value = 0;
        for (int i = 0; i < 4; ++i) {
            const char hex = text_[position_++];
            value <<= 4;
            if (hex >= '0' && hex <= '9') {
                value += static_cast<unsigned int>(hex - '0');
            } else if (hex >= 'a' && hex <= 'f') {
                value += static_cast<unsigned int>(hex - 'a' + 10);
            } else if (hex >= 'A' && hex <= 'F') {
                value += static_cast<unsigned int>(hex - 'A' + 10);
            } else {
                return false;
            }
        }
        return true;
    }

    static void AppendUtf8(unsigned int code_point, std::string& output) {
        if (code_point <= 0x7F) {
            output.push_back(static_cast<char>(code_point));
        } else if (code_point <= 0x7FF) {
            output.push_back(static_cast<char>(0xC0 | (code_point >> 6)));
            output.push_back(static_cast<char>(0x80 | (code_point & 0x3F)));
        } else if (code_point <= 0xFFFF) {
            output.push_back(static_cast<char>(0xE0 | (code_point >> 12)));
            output.push_back(static_cast<char>(0x80 | ((code_point >> 6) & 0x3F)));
            output.push_back(static_cast<char>(0x80 | (code_point & 0x3F)));
        } else {
            output.push_back(static_cast<char>(0xF0 | (code_point >> 18)));
            output.push_back(static_cast<char>(0x80 | ((code_point >> 12) & 0x3F)));
            output.push_back(static_cast<char>(0x80 | ((code_point >> 6) & 0x3F)));
            output.push_back(static_cast<char>(0x80 | (code_point & 0x3F)));
        }
    }

    static std::string EncodeJsonString(const std::string& decoded) {
        static constexpr char kHex[] = "0123456789ABCDEF";
        std::string encoded;
        encoded.reserve(decoded.size() + 2);
        encoded.push_back('"');
        for (const unsigned char c : decoded) {
            switch (c) {
                case '"': encoded += "\\\""; break;
                case '\\': encoded += "\\\\"; break;
                case '\b': encoded += "\\b"; break;
                case '\f': encoded += "\\f"; break;
                case '\n': encoded += "\\n"; break;
                case '\r': encoded += "\\r"; break;
                case '\t': encoded += "\\t"; break;
                default:
                    if (c < 0x20) {
                        encoded += "\\u00";
                        encoded.push_back(kHex[(c >> 4) & 0x0F]);
                        encoded.push_back(kHex[c & 0x0F]);
                    } else {
                        encoded.push_back(static_cast<char>(c));
                    }
                    break;
            }
        }
        encoded.push_back('"');
        return encoded;
    }

    void BuildCanonicalizedDocument(std::string& canonicalized) const {
        canonicalized.clear();
        canonicalized.reserve(text_.size());
        std::size_t cursor = 0;
        for (const MemberNameReplacement& replacement : member_name_replacements_) {
            canonicalized.append(text_, cursor, replacement.begin - cursor);
            canonicalized += EncodeJsonString(replacement.decoded);
            cursor = replacement.end;
        }
        canonicalized.append(text_, cursor, std::string::npos);
    }

    bool ParseNumber() {
        const std::size_t start = position_;
        Consume('-');
        if (position_ >= text_.size()) {
            return false;
        }
        if (text_[position_] == '0') {
            ++position_;
            if (position_ < text_.size() && text_[position_] >= '0' && text_[position_] <= '9') {
                return false;
            }
        } else {
            if (text_[position_] < '1' || text_[position_] > '9') {
                return false;
            }
            while (position_ < text_.size() && text_[position_] >= '0' && text_[position_] <= '9') {
                ++position_;
            }
        }
        if (Consume('.')) {
            const std::size_t fraction_start = position_;
            while (position_ < text_.size() && text_[position_] >= '0' && text_[position_] <= '9') {
                ++position_;
            }
            if (position_ == fraction_start) {
                return false;
            }
        }
        if (position_ < text_.size() && (text_[position_] == 'e' || text_[position_] == 'E')) {
            ++position_;
            if (position_ < text_.size() && (text_[position_] == '+' || text_[position_] == '-')) {
                ++position_;
            }
            const std::size_t exponent_start = position_;
            while (position_ < text_.size() && text_[position_] >= '0' && text_[position_] <= '9') {
                ++position_;
            }
            if (position_ == exponent_start) {
                return false;
            }
        }
        return position_ > start;
    }

    bool ConsumeLiteral(const char* literal) {
        const std::size_t start = position_;
        for (std::size_t i = 0; literal[i] != '\0'; ++i) {
            if (position_ >= text_.size() || text_[position_] != literal[i]) {
                position_ = start;
                return false;
            }
            ++position_;
        }
        return true;
    }

    bool Consume(char expected) {
        if (position_ >= text_.size() || text_[position_] != expected) {
            return false;
        }
        ++position_;
        return true;
    }

    void SkipWhitespace() {
        while (position_ < text_.size()) {
            const char c = text_[position_];
            if (c != ' ' && c != '\t' && c != '\n' && c != '\r') {
                break;
            }
            ++position_;
        }
    }

    const std::string& text_;
    std::size_t position_ = 0;
    std::vector<MemberNameReplacement> member_name_replacements_{};
};

// OpenCV FileStorage 会在节点创建阶段把超出 int 范围的裸整数饱和为 int，
// 导致后续 ReadIntegerValue 无法观察原值。严格语法通过后，仅把这些裸整数
// 改写为等值的 JSON 实数写法，使 OpenCV 保留 double 值；其他 token 和字符串不变。
std::string PreserveOutOfRangeIntegerValues(const std::string& text) {
    std::string output;
    output.reserve(text.size());
    bool in_string = false;
    bool escaped = false;
    for (std::size_t i = 0; i < text.size();) {
        const char c = text[i];
        if (in_string) {
            output.push_back(c);
            ++i;
            if (escaped) {
                escaped = false;
            } else if (c == '\\') {
                escaped = true;
            } else if (c == '"') {
                in_string = false;
            }
            continue;
        }
        if (c == '"') {
            in_string = true;
            output.push_back(c);
            ++i;
            continue;
        }
        if (c != '-' && (c < '0' || c > '9')) {
            output.push_back(c);
            ++i;
            continue;
        }

        const std::size_t start = i;
        if (text[i] == '-') {
            ++i;
        }
        while (i < text.size() && text[i] >= '0' && text[i] <= '9') {
            ++i;
        }
        bool is_bare_integer = true;
        if (i < text.size() && text[i] == '.') {
            is_bare_integer = false;
            ++i;
            while (i < text.size() && text[i] >= '0' && text[i] <= '9') {
                ++i;
            }
        }
        if (i < text.size() && (text[i] == 'e' || text[i] == 'E')) {
            is_bare_integer = false;
            ++i;
            if (i < text.size() && (text[i] == '+' || text[i] == '-')) {
                ++i;
            }
            while (i < text.size() && text[i] >= '0' && text[i] <= '9') {
                ++i;
            }
        }
        const std::string token = text.substr(start, i - start);
        output += token;
        if (is_bare_integer) {
            const double numeric = std::strtod(token.c_str(), nullptr);
            if (!std::isfinite(numeric) ||
                numeric < static_cast<double>(std::numeric_limits<int>::min()) ||
                numeric > static_cast<double>(std::numeric_limits<int>::max())) {
                output += ".0";
            }
        }
    }
    return output;
}

/**
 * 基于 OpenCV FileStorage 解析 JSON 字符串为结构化节点树。
 * 先剥离注释再解析，确保解析结果是一个非空的 JSON 对象。
 * @param text 原始 JSON 字符串（可含注释）
 * @param storage 输出参数，解析后的 FileStorage 对象
 * @return true 表示解析成功且根节点为非空 Map
 */
bool ParseJsonObject(const std::string& text, cv::FileStorage& storage) {
    std::string sanitized;
    if (!StripJsonComments(text, sanitized)) {
        return false;
    }
    std::string canonicalized;
    if (!StrictJsonObjectSyntax(sanitized).ParseDocument(canonicalized)) {
        return false;
    }
    const std::size_t first = canonicalized.find_first_not_of(" \t\n\r");
    const std::size_t last = canonicalized.find_last_not_of(" \t\n\r");
    const std::string opencv_input = PreserveOutOfRangeIntegerValues(
        canonicalized.substr(first, last - first + 1));
    try {
        if (!storage.open(opencv_input,
                          cv::FileStorage::READ | cv::FileStorage::MEMORY |
                              cv::FileStorage::FORMAT_JSON)) {
            return false;
        }
    } catch (...) {
        return false;
    }
    const cv::FileNode root = storage.root();
    return !root.empty() && root.isMap();
}

/**
 * 从 JSON 节点读取数值（整数或浮点数）。
 * @param node OpenCV JSON 节点
 * @param value 输出参数，读取到的数值
 * @return true 表示读取成功
 */
bool ReadNumberNode(const cv::FileNode& node, double& value) {
    if (node.empty() || (!node.isInt() && !node.isReal())) {
        return false;
    }
    value = static_cast<double>(node.real());
    return true;
}

/**
 * 读取必填数值参数，节点缺失或类型不匹配则返回 false。
 * @param root JSON 根节点
 * @param key 参数键名
 * @param value 输出参数，读取到的数值
 * @return true 表示读取成功
 */
bool ReadRequiredNumber(const cv::FileNode& root, const char* key, double& value) {
    return ReadNumberNode(root[key], value);
}

/**
 * 读取整数值 —— 数值必须有限、数学上为整数且位于 int 的可表示范围内。
 * @param node OpenCV JSON 节点
 * @param value 输出参数，读取到的整数值
 * @return true 表示读取成功且可以无损转换为 int
 */
bool ReadIntegerValue(const cv::FileNode& node, int& value) {
    double numeric = 0.0;
    if (!ReadNumberNode(node, numeric)) {
        return false;
    }
    if (!std::isfinite(numeric) || std::trunc(numeric) != numeric ||
        numeric < static_cast<double>(std::numeric_limits<int>::min()) ||
        numeric > static_cast<double>(std::numeric_limits<int>::max())) {
        return false;
    }
    value = static_cast<int>(numeric);
    return true;
}

/**
 * 读取布尔值，支持整数（非零为 true）和字符串表示（true/TRUE/1/yes/on/false/FALSE/0/no/off）。
 * @param node OpenCV JSON 节点
 * @param value 输出参数，读取到的布尔值
 * @return true 表示读取成功
 */
bool ReadBoolValue(const cv::FileNode& node, bool& value) {
    if (node.empty()) {
        return false;
    }
    if (node.isInt()) {
        value = static_cast<int>(node.real()) != 0;
        return true;
    }
    if (node.isString()) {
        const std::string text = static_cast<std::string>(node);
        if (text == "true" || text == "TRUE" || text == "1" || text == "yes" || text == "on") {
            value = true;
            return true;
        }
        if (text == "false" || text == "FALSE" || text == "0" || text == "no" || text == "off") {
            value = false;
            return true;
        }
    }
    return false;
}

/**
 * 读取字符串值。
 * @param node OpenCV JSON 节点
 * @param value 输出参数，读取到的字符串
 * @return true 表示读取成功（节点非空且为字符串类型）
 */
bool ReadStringValue(const cv::FileNode& node, std::string& value) {
    if (node.empty() || !node.isString()) {
        return false;
    }
    value = static_cast<std::string>(node);
    return true;
}

/**
 * 读取可选数值参数 —— 键缺失时不报错（保留默认值），格式错误时设置 malformed 标志。
 * @param root JSON 根节点
 * @param key 参数键名
 * @param value 输出参数，读取到的数值（若缺失则不变）
 * @param malformed 输出参数，格式错误时置为 true
 */
void ReadOptionalNumber(const cv::FileNode& root, const char* key, double& value, bool& malformed) {
    const cv::FileNode node = root[key];
    if (node.empty()) {
        return;
    }
    if (!ReadNumberNode(node, value)) {
        malformed = true;
    }
}

void ReadOptionalNonNegativeFiniteNumber(const cv::FileNode& root,
                                         const char* key,
                                         double& value,
                                         bool& malformed) {
    const cv::FileNode node = root[key];
    if (node.empty()) {
        return;
    }
    double parsed = value;
    if (!ReadNumberNode(node, parsed) || !std::isfinite(parsed) || parsed < 0.0) {
        malformed = true;
        return;
    }
    value = parsed;
}

/**
 * 读取可选整数参数。
 * @param root JSON 根节点
 * @param key 参数键名
 * @param value 输出参数，读取到的整数值
 * @param malformed 格式错误时置为 true
 */
void ReadOptionalInt(const cv::FileNode& root, const char* key, int& value, bool& malformed) {
    const cv::FileNode node = root[key];
    if (node.empty()) {
        return;
    }
    if (!ReadIntegerValue(node, value)) {
        malformed = true;
    }
}

/**
 * 读取可选布尔参数。
 * @param root JSON 根节点
 * @param key 参数键名
 * @param value 输出参数，读取到的布尔值
 * @param malformed 格式错误时置为 true
 */
void ReadOptionalBool(const cv::FileNode& root, const char* key, bool& value, bool& malformed) {
    const cv::FileNode node = root[key];
    if (node.empty()) {
        return;
    }
    if (!ReadBoolValue(node, value)) {
        malformed = true;
    }
}

/**
 * 读取嵌套可选数值参数（const char* child 版本）。
 * @param root JSON 根节点
 * @param parent 父级键名
 * @param child 子级键名
 * @param value 输出参数，读取到的数值
 * @param malformed 格式错误时置为 true
 */
void ReadOptionalNestedNumber(const cv::FileNode& root,
                              const char* parent,
                              const char* child,
                              double& value,
                              bool& malformed) {
    const cv::FileNode parent_node = root[parent];
    if (parent_node.empty()) {
        return;
    }
    if (!parent_node.isMap()) {
        malformed = true;
        return;
    }
    const cv::FileNode node = parent_node[child];
    if (node.empty()) {
        return;
    }
    if (!ReadNumberNode(node, value)) {
        malformed = true;
    }
}

/**
 * 读取嵌套可选数值参数（std::string child 版本）。
 * @param root JSON 根节点
 * @param parent 父级键名
 * @param child 子级键名
 * @param value 输出参数，读取到的数值
 * @param malformed 格式错误时置为 true
 */
void ReadOptionalNestedNumber(const cv::FileNode& root,
                              const char* parent,
                              const std::string& child,
                              double& value,
                              bool& malformed) {
    const cv::FileNode parent_node = root[parent];
    if (parent_node.empty()) {
        return;
    }
    if (!parent_node.isMap()) {
        malformed = true;
        return;
    }
    const cv::FileNode node = parent_node[child];
    if (node.empty()) {
        return;
    }
    if (!ReadNumberNode(node, value)) {
        malformed = true;
    }
}

/**
 * 读取嵌套可选数值参数（float 特化，const char* child 版本）。
 * 通过 double 中转读取后再转换为 float。
 * @param root JSON 根节点
 * @param parent 父级键名
 * @param child 子级键名
 * @param value 输出参数，读取到的浮点值
 * @param malformed 格式错误时置为 true
 */
void ReadOptionalNestedNumber(const cv::FileNode& root,
                              const char* parent,
                              const char* child,
                              float& value,
                              bool& malformed) {
    double temporary = static_cast<double>(value);
    ReadOptionalNestedNumber(root, parent, child, temporary, malformed);
    value = static_cast<float>(temporary);
}

/**
 * 读取嵌套可选数值参数（float 特化，std::string child 版本）。
 * @param root JSON 根节点
 * @param parent 父级键名
 * @param child 子级键名
 * @param value 输出参数，读取到的浮点值
 * @param malformed 格式错误时置为 true
 */
void ReadOptionalNestedNumber(const cv::FileNode& root,
                              const char* parent,
                              const std::string& child,
                              float& value,
                              bool& malformed) {
    double temporary = static_cast<double>(value);
    ReadOptionalNestedNumber(root, parent, child, temporary, malformed);
    value = static_cast<float>(temporary);
}

/**
 * 读取嵌套可选浮点数组（固定长度 N）。
 * @tparam N 数组长度
 * @param root JSON 根节点
 * @param parent 父级键名
 * @param child 子级键名
 * @param values 输出参数，读取到的浮点数组
 * @param malformed 格式错误时置为 true
 */
template <std::size_t N>
void ReadOptionalNestedFloatArray(const cv::FileNode& root,
                                  const char* parent,
                                  const char* child,
                                  std::array<float, N>& values,
                                  bool& malformed) {
    const cv::FileNode parent_node = root[parent];
    if (parent_node.empty()) {
        return;
    }
    if (!parent_node.isMap()) {
        malformed = true;
        return;
    }
    const cv::FileNode node = parent_node[child];
    if (node.empty()) {
        return;
    }
    if (!node.isSeq() || node.size() != N) {
        malformed = true;
        return;
    }
    for (std::size_t index = 0; index < N; ++index) {
        double value = 0.0;
        if (!ReadNumberNode(node[static_cast<int>(index)], value)) {
            malformed = true;
            return;
        }
        values[index] = static_cast<float>(value);
    }
}

/**
 * 读取嵌套可选布尔值。
 * @param root JSON 根节点
 * @param parent 父级键名
 * @param child 子级键名
 * @param value 输出参数，读取到的布尔值
 * @param malformed 格式错误时置为 true
 */
void ReadOptionalNestedBool(const cv::FileNode& root,
                            const char* parent,
                            const char* child,
                            bool& value,
                            bool& malformed) {
    const cv::FileNode parent_node = root[parent];
    if (parent_node.empty()) {
        return;
    }
    if (!parent_node.isMap()) {
        malformed = true;
        return;
    }
    const cv::FileNode node = parent_node[child];
    if (node.empty()) {
        return;
    }
    if (!ReadBoolValue(node, value)) {
        malformed = true;
    }
}

/**
 * 读取嵌套可选整数。
 * @param root JSON 根节点
 * @param parent 父级键名
 * @param child 子级键名
 * @param value 输出参数，读取到的整数值
 * @param malformed 格式错误时置为 true
 */
void ReadOptionalNestedInt(const cv::FileNode& root,
                           const char* parent,
                           const char* child,
                           int& value,
                           bool& malformed) {
    const cv::FileNode parent_node = root[parent];
    if (parent_node.empty()) {
        return;
    }
    if (!parent_node.isMap()) {
        malformed = true;
        return;
    }
    const cv::FileNode node = parent_node[child];
    if (node.empty()) {
        return;
    }
    if (!ReadIntegerValue(node, value)) {
        malformed = true;
    }
}

/**
 * 读取嵌套可选字符串。
 * @param root JSON 根节点
 * @param parent 父级键名
 * @param child 子级键名
 * @param value 输出参数，读取到的字符串值
 * @param malformed 格式错误时置为 true
 */
void ReadOptionalNestedString(const cv::FileNode& root,
                              const char* parent,
                              const char* child,
                              std::string& value,
                              bool& malformed) {
    const cv::FileNode parent_node = root[parent];
    if (parent_node.empty()) {
        return;
    }
    if (!parent_node.isMap()) {
        malformed = true;
        return;
    }
    const cv::FileNode node = parent_node[child];
    if (node.empty()) {
        return;
    }
    if (!ReadStringValue(node, value)) {
        malformed = true;
    }
}

/**
 * 读取必填嵌套数值参数（缺失或类型不匹配返回 false）。
 * @param root JSON 根节点
 * @param parent 父级键名
 * @param child 子级键名
 * @param value 输出参数，读取到的数值
 * @return true 表示读取成功
 */
bool ReadRequiredNestedNumber(const cv::FileNode& root,
                              const char* parent,
                              const char* child,
                              double& value) {
    const cv::FileNode parent_node = root[parent];
    if (parent_node.empty() || !parent_node.isMap()) {
        return false;
    }
    return ReadNumberNode(parent_node[child], value);
}

/**
 * 读取必填嵌套字符串值。
 * @param root JSON 根节点
 * @param parent 父级键名
 * @param child 子级键名
 * @param value 输出参数，读取到的字符串
 * @return true 表示读取成功
 */
bool ReadRequiredNestedString(const cv::FileNode& root,
                              const char* parent,
                              const char* child,
                              std::string& value) {
    const cv::FileNode parent_node = root[parent];
    if (parent_node.empty() || !parent_node.isMap()) {
        return false;
    }
    return ReadStringValue(parent_node[child], value);
}

/**
 * 读取必填嵌套整数值。
 * @param root JSON 根节点
 * @param parent 父级键名
 * @param child 子级键名
 * @param value 输出参数，读取到的整数值
 * @return true 表示读取成功
 */
bool ReadRequiredNestedInt(const cv::FileNode& root,
                           const char* parent,
                           const char* child,
                           int& value) {
    const cv::FileNode parent_node = root[parent];
    if (parent_node.empty() || !parent_node.isMap()) {
        return false;
    }
    return ReadIntegerValue(parent_node[child], value);
}

/**
 * 检查数值是否为有限值且处于闭区间 [min_value, max_value] 内。
 * @param value 待检查的数值
 * @param min_value 区间下限
 * @param max_value 区间上限
 * @return true 表示数值有限且在区间范围内
 */
bool IsFiniteInRange(double value, double min_value, double max_value) {
    return std::isfinite(value) && value >= min_value && value <= max_value;
}

/**
 * 校验 BEV 控制模型参数是否在合理范围内。
 * @param params BEV 控制模型参数结构体
 * @return true 表示所有参数均通过合法性校验
 */
bool ValidateBEVControlModel(const port::BEVControlModelParameters& params) {
    return IsFiniteInRange(params.lateral_offset_to_wheel_delta_gain, 0.0, 1000.0) &&
           IsFiniteInRange(params.heading_error_to_wheel_delta_gain, 0.0, 1000.0) &&
           IsFiniteInRange(params.curvature_to_wheel_delta_gain, 0.0, 1000.0) &&
           params.tracking_fit_min_samples >= 3 &&
           params.tracking_fit_min_samples <= static_cast<int>(port::kBevReferenceSampleCount);
}

bool ValidateBEVGeometry(const port::BEVGeometryParameters& params) {
    return IsFiniteInRange(params.nominal_road_half_width_m, 0.01, 2.0) &&
           IsFiniteInRange(params.boundary_trace_max_adjacent_distance_m, 1.0e-6, 1000.0) &&
           params.sparse_row_count >= 1 &&
           params.sparse_row_count <= static_cast<int>(port::kBevReferenceSampleCount);
}

bool ValidateBEVElement(const port::BEVElementParameters& params) {
    return params.cross_min_sampleable_per_row >= 1 &&
           params.cross_connectivity_sample_index >= 0 &&
           params.cross_connectivity_sample_index <
               static_cast<int>(port::kBevReferenceSampleCount) &&
           IsFiniteInRange(params.cross_boundary_expansion_min_m, 1.0e-6, 2.0) &&
           IsFiniteInRange(params.circle_v2_normal_trace_start_yaw_deg, 1.0, 720.0) &&
           IsFiniteInRange(params.circle_v2_exit_trace_start_yaw_deg, 1.0, 720.0) &&
           IsFiniteInRange(params.circle_v2_calm_fallback_yaw_deg, 1.0, 720.0) &&
           params.circle_v2_normal_trace_start_yaw_deg <
               params.circle_v2_exit_trace_start_yaw_deg &&
           params.circle_v2_exit_trace_start_yaw_deg <
               params.circle_v2_calm_fallback_yaw_deg &&
           params.circle_v2_calm_trace_ms >= 1 &&
           params.circle_v2_cooldown_ms >= 0 &&
           params.circle_v2_inner_trace_stall_timeout_ms >= 1 &&
           IsFiniteInRange(params.circle_v2_inner_trace_stall_yaw_min_deg, 0.0, 720.0) &&
           IsFiniteInRange(params.circle_v2_inner_trace_path_offset_m, 0.0, 2.0) &&
           IsFiniteInRange(params.circle_v2_opposite_straight_confidence_min, 0.0, 1.0) &&
           IsFiniteInRange(params.circle_v2_min_sampleable_width_m, 1.0e-6, 10.0) &&
           IsFiniteInRange(params.circle_v2_opening_forward_min_m, 0.0, 2.0) &&
           IsFiniteInRange(params.circle_v2_opening_forward_max_m, 0.0, 2.0) &&
           params.circle_v2_opening_forward_max_m >= params.circle_v2_opening_forward_min_m &&
           IsFiniteInRange(params.circle_v2_opening_distance_min_m, 1.0e-6, 2.0) &&
           IsFiniteInRange(params.circle_v2_opening_confirm_forward_span_m, 1.0e-6, 2.0) &&
           IsFiniteInRange(params.circle_v2_entry_forward_min_m, 0.0, 2.0) &&
           IsFiniteInRange(params.circle_v2_entry_forward_max_m, 0.0, 2.0) &&
           params.circle_v2_entry_forward_max_m >= params.circle_v2_entry_forward_min_m &&
           IsFiniteInRange(params.circle_v2_inner_geometry_forward_min_m, 0.0, 2.0) &&
           IsFiniteInRange(params.circle_v2_inner_geometry_forward_max_m, 0.0, 2.0) &&
           params.circle_v2_inner_geometry_forward_max_m >=
               params.circle_v2_inner_geometry_forward_min_m &&
           IsFiniteInRange(params.circle_v2_exit_geometry_forward_min_m, 0.0, 2.0) &&
           IsFiniteInRange(params.circle_v2_exit_geometry_forward_max_m, 0.0, 2.0) &&
           params.circle_v2_exit_geometry_forward_max_m >=
               params.circle_v2_exit_geometry_forward_min_m &&
           IsFiniteInRange(params.circle_v2_exit_straight_max_lateral_span_m, 1.0e-6, 2.0) &&
           IsFiniteInRange(params.circle_v2_exit_tangent_fit_span_m, 1.0e-6, 2.0);
}

bool ValidateReferenceTimeAlignment(const port::ReferenceTimeAlignmentParameters& params,
                                    const port::MotionOdometryParameters& odometry) {
    return port::ValidateReferenceTimeAlignmentParameters(params, odometry);
}

bool ValidateCameraSource(const port::CameraSourceParameters& params) {
    return params.backend == "v4l2_yuyv" &&
           !params.device.empty() &&
           params.width > 0 &&
           params.height > 0 &&
           params.width <= port::kCompiledCameraFrameWidth &&
           params.height <= port::kCompiledCameraFrameHeight &&
           params.fps >= 1 &&
           params.buffer_count >= 2 &&
           params.poll_timeout_ms >= 1;
}

/**
 * 读取必填字符串值。
 * @param root JSON 根节点
 * @param key 参数键名
 * @param value 输出参数，读取到的字符串
 * @return true 表示读取成功
 */
bool ReadRequiredString(const cv::FileNode& root, const char* key, std::string& value) {
    const cv::FileNode node = root[key];
    if (node.empty() || !node.isString()) {
        return false;
    }
    value = static_cast<std::string>(node);
    return true;
}

/**
 * 解析子系统模式文本枚举。
 * 支持 "adaptation-hook"、"disabled"、"direct-match" 三种文本到枚举值的映射。
 * @param mode_text 模式文本字符串
 * @param mode 输出参数，解析后的子系统模式枚举值
 * @return true 表示解析成功
 */
bool ParseMode(const std::string& mode_text, port::SubsystemMode& mode) {
    if (mode_text == "adaptation-hook") {
        mode = port::SubsystemMode::kAdaptationHook;
        return true;
    }
    if (mode_text == "disabled") {
        mode = port::SubsystemMode::kDisabled;
        return true;
    }
    if (mode_text == "direct-match") {
        mode = port::SubsystemMode::kDirectMatch;
        return true;
    }
    return false;
}

/**
 * 解析硬件配置文件中单个子系统的配置块（包含 mode 和 hook 字段）。
 * @param root JSON 根节点
 * @param key 子系统键名（如 "camera"、"actuator" 等）
 * @param out_profile 输出参数，解析后的子系统配置
 * @return true 表示解析成功
 */
bool ParseProfileBlock(const cv::FileNode& root,
                       const char* key,
                       port::SubsystemProfile& out_profile) {
    const cv::FileNode block = root[key];
    if (block.empty() || !block.isMap()) {
        return false;
    }

    std::string mode_text;
    std::string hook_name;
    if (!ReadRequiredString(block, "mode", mode_text) || !ReadRequiredString(block, "hook", hook_name)) {
        return false;
    }
    port::SubsystemMode parsed_mode = port::SubsystemMode::kDirectMatch;
    if (!ParseMode(mode_text, parsed_mode)) {
        return false;
    }
    out_profile.mode = parsed_mode;
    out_profile.hook = hook_name;
    return true;
}

/**
 * 生成子系统解析错误详情字符串。
 * @param key 子系统键名
 * @return 格式化的错误描述字符串
 */
std::string ProfileBlockError(const char* key) {
    return std::string("hardware profile parse failure for subsystem '") + key +
           "' (missing block or malformed mode/hook)";
}

bool ReadRequiredRuntimeParams(const cv::FileNode& root, port::RuntimeParameters& parsed) {
    bool all_ok = true;
    all_ok &= ReadRequiredNumber(root, "RUNNING_SPEED_TARGET", parsed.running_speed_target);
    all_ok &= std::isfinite(parsed.running_speed_target) &&
              parsed.running_speed_target >= 0.0 &&
              parsed.running_speed_target <= control::kWheelSpeedTargetMax;
    all_ok &= ReadRequiredNestedNumber(root, "YAW_RATE_PID", "D", parsed.yaw_rate_pid_d);
    all_ok &= ReadRequiredNestedNumber(root, "LEFT_WHEEL_PID", "P", parsed.left_wheel_pid.p);
    all_ok &= ReadRequiredNestedNumber(root, "LEFT_WHEEL_PID", "I", parsed.left_wheel_pid.i);
    all_ok &= ReadRequiredNestedNumber(root, "LEFT_WHEEL_PID", "D", parsed.left_wheel_pid.d);
    all_ok &= ReadRequiredNestedNumber(
        root, "LEFT_WHEEL_PID", "INTEGRAL_LIMIT", parsed.left_wheel_pid.integral_limit);
    all_ok &= ReadRequiredNestedNumber(root, "RIGHT_WHEEL_PID", "P", parsed.right_wheel_pid.p);
    all_ok &= ReadRequiredNestedNumber(root, "RIGHT_WHEEL_PID", "I", parsed.right_wheel_pid.i);
    all_ok &= ReadRequiredNestedNumber(root, "RIGHT_WHEEL_PID", "D", parsed.right_wheel_pid.d);
    all_ok &= ReadRequiredNestedNumber(
        root, "RIGHT_WHEEL_PID", "INTEGRAL_LIMIT", parsed.right_wheel_pid.integral_limit);
    const auto wheel_pid_values_valid = [](const port::WheelPidParameters& params) {
        return std::isfinite(params.p) && std::isfinite(params.i) &&
               std::isfinite(params.d) && std::isfinite(params.integral_limit) &&
               params.integral_limit >= 0.0;
    };
    all_ok &= wheel_pid_values_valid(parsed.left_wheel_pid);
    all_ok &= wheel_pid_values_valid(parsed.right_wheel_pid);
    all_ok &= ReadRequiredNestedString(root, "assistant_tcp", "host", parsed.assistant_tcp.host);
    all_ok &= ReadRequiredNestedInt(root, "assistant_tcp", "port", parsed.assistant_tcp.port);
    all_ok &= parsed.assistant_tcp.port >= port::kMinimumTcpPort &&
              parsed.assistant_tcp.port <= port::kMaximumTcpPort;
    return all_ok;
}

void ReadControlParams(const cv::FileNode& root, port::RuntimeParameters& parsed, bool& optional_malformed) {
    ReadOptionalInt(root, "low_voltage_raw_threshold", parsed.low_voltage_raw_threshold, optional_malformed);
    ReadOptionalInt(root, "control_period_ms", parsed.control_period_ms, optional_malformed);
    ReadOptionalInt(root, "perception_stale_ms", parsed.perception_stale_ms, optional_malformed);
    ReadOptionalInt(root, "pwm_limit", parsed.pwm_limit, optional_malformed);
    ReadOptionalInt(root, "raw_turn_output_limit", parsed.raw_turn_output_limit, optional_malformed);
    ReadOptionalNonNegativeFiniteNumber(
        root, "wheel_turn_accel_delta_scale", parsed.wheel_turn_accel_delta_scale, optional_malformed);
    ReadOptionalNonNegativeFiniteNumber(
        root, "wheel_turn_decel_delta_scale", parsed.wheel_turn_decel_delta_scale, optional_malformed);
    ReadOptionalInt(root, "pwm_floor", parsed.pwm_floor, optional_malformed);
    ReadOptionalBool(root, "prohibit_reverse_pwm", parsed.prohibit_reverse_pwm, optional_malformed);
    ReadOptionalInt(root, "drive_pwm_step_limit", parsed.drive_pwm_step_limit, optional_malformed);
    if (parsed.low_voltage_raw_threshold <= 0 ||
        parsed.control_period_ms <= 0 ||
        parsed.control_period_ms > port::kMaximumControlPeriodMs ||
        parsed.perception_stale_ms <= 0 ||
        parsed.perception_stale_ms > port::kMaximumRuntimeIntervalMs ||
        parsed.raw_turn_output_limit < 0 ||
        parsed.pwm_limit <= 0 ||
        parsed.pwm_limit > port::kDrivePwmDutyCapability ||
        parsed.pwm_floor < 0 ||
        parsed.pwm_floor > parsed.pwm_limit ||
        parsed.drive_pwm_step_limit <= 0 ||
        parsed.drive_pwm_step_limit > parsed.pwm_limit) {
        optional_malformed = true;
    }
    ReadOptionalBool(
        root, "brushless_debug_fixed_pwm_enabled", parsed.brushless_debug_fixed_pwm_enabled, optional_malformed);
    ReadOptionalInt(root, "brushless_debug_fixed_pwm", parsed.brushless_debug_fixed_pwm, optional_malformed);
    if (parsed.brushless_debug_fixed_pwm < 0 || parsed.brushless_debug_fixed_pwm > 1000) {
        optional_malformed = true;
    }
    ReadOptionalInt(root, "motion_unveto_confirm_cycles", parsed.motion_unveto_confirm_cycles, optional_malformed);
    ReadOptionalInt(root, "motion_spinup_ms", parsed.motion_spinup_ms, optional_malformed);
    ReadOptionalNumber(root, "motion_turn_limit_spinup", parsed.motion_turn_limit_spinup, optional_malformed);
    ReadOptionalInt(root, "motion_stop_ms", parsed.motion_stop_ms, optional_malformed);
    ReadOptionalInt(
        root, "motion_stop_encoder_threshold", parsed.motion_stop_encoder_threshold, optional_malformed);
    ReadOptionalInt(root, "motion_fault_rearm_hold_ms", parsed.motion_fault_rearm_hold_ms, optional_malformed);
    ReadOptionalInt(
        root, "control_snapshot_emit_interval_ms", parsed.control_snapshot_emit_interval_ms, optional_malformed);
    const int maximum_confirm_cycles =
        parsed.control_period_ms > 0
            ? port::kMaximumRuntimeIntervalMs / parsed.control_period_ms
            : 0;
    if (parsed.motion_unveto_confirm_cycles <= 0 ||
        parsed.motion_unveto_confirm_cycles > maximum_confirm_cycles ||
        parsed.motion_spinup_ms < 0 ||
        parsed.motion_spinup_ms > port::kMaximumRuntimeIntervalMs ||
        !std::isfinite(parsed.motion_turn_limit_spinup) ||
        parsed.motion_turn_limit_spinup < 0.0 ||
        parsed.motion_turn_limit_spinup > 1.0 ||
        parsed.motion_stop_ms < 0 ||
        parsed.motion_stop_ms > port::kMaximumRuntimeIntervalMs ||
        parsed.motion_stop_encoder_threshold < 0 ||
        parsed.motion_stop_encoder_threshold > control::kWheelSpeedTargetMax ||
        parsed.motion_fault_rearm_hold_ms < 0 ||
        parsed.motion_fault_rearm_hold_ms > port::kMaximumRuntimeIntervalMs ||
        parsed.control_snapshot_emit_interval_ms <= 0 ||
        parsed.control_snapshot_emit_interval_ms > port::kMaximumRuntimeIntervalMs) {
        optional_malformed = true;
    }
    ReadOptionalNestedNumber(root, "YAW_RATE_PID", "P", parsed.yaw_rate_pid_p, optional_malformed);
    ReadOptionalNestedNumber(root, "YAW_RATE_PID", "I", parsed.yaw_rate_pid_i, optional_malformed);
    if (!control::YawRatePidArithmeticIsFinite(parsed.yaw_rate_pid_p,
                                               parsed.yaw_rate_pid_i,
                                               parsed.yaw_rate_pid_d)) {
        optional_malformed = true;
    }
    ReadOptionalNestedNumber(
        root, "LEFT_WHEEL_PID", "MEASUREMENT_FILTER_ALPHA", parsed.left_wheel_pid.measurement_filter_alpha,
        optional_malformed);
    ReadOptionalNestedNumber(
        root, "RIGHT_WHEEL_PID", "MEASUREMENT_FILTER_ALPHA", parsed.right_wheel_pid.measurement_filter_alpha,
        optional_malformed);
    if (!std::isfinite(parsed.left_wheel_pid.measurement_filter_alpha) ||
        parsed.left_wheel_pid.measurement_filter_alpha < 0.0 ||
        parsed.left_wheel_pid.measurement_filter_alpha > 1.0 ||
        !std::isfinite(parsed.right_wheel_pid.measurement_filter_alpha) ||
        parsed.right_wheel_pid.measurement_filter_alpha < 0.0 ||
        parsed.right_wheel_pid.measurement_filter_alpha > 1.0) {
        optional_malformed = true;
    }
}

void ReadMediaParams(const cv::FileNode& root, port::RuntimeParameters& parsed, bool& optional_malformed) {
    ReadOptionalBool(root, "assistant_enabled", parsed.assistant_enabled, optional_malformed);
    ReadOptionalBool(root, "steering_media_enabled", parsed.steering_media_enabled, optional_malformed);
    ReadOptionalInt(root, "steering_media_port", parsed.steering_media_port, optional_malformed);
    ReadOptionalInt(
        root, "steering_media_publish_interval_ms", parsed.steering_media_publish_interval_ms, optional_malformed);
    ReadOptionalInt(root, "steering_media_downsample", parsed.steering_media_downsample, optional_malformed);
    if (parsed.steering_media_downsample < 1 || parsed.steering_media_downsample > 8) {
        optional_malformed = true;
    }
    ReadOptionalBool(
        root, "steering_media_publish_latest_frame", parsed.steering_media_publish_latest_frame,
        optional_malformed);
    ReadOptionalInt(root, "steering_media_gray_bits", parsed.steering_media_gray_bits, optional_malformed);
    if (parsed.steering_media_gray_bits != 1 && parsed.steering_media_gray_bits != 2 &&
        parsed.steering_media_gray_bits != 4 && parsed.steering_media_gray_bits != 8) {
        optional_malformed = true;
    }
    ReadOptionalBool(root, "steering_media_publish_disarmed", parsed.steering_media_publish_disarmed,
                     optional_malformed);
    ReadOptionalInt(root, "low_voltage_sample_interval_ms", parsed.low_voltage_sample_interval_ms,
                    optional_malformed);
    if (parsed.steering_media_port < port::kMinimumTcpPort ||
        parsed.steering_media_port > port::kMaximumTcpPort ||
        parsed.steering_media_publish_interval_ms < 0 ||
        parsed.steering_media_publish_interval_ms > port::kMaximumRuntimeIntervalMs ||
        parsed.low_voltage_sample_interval_ms <= 0 ||
        parsed.low_voltage_sample_interval_ms > port::kMaximumRuntimeIntervalMs) {
        optional_malformed = true;
    }
}

void ReadMlParams(const cv::FileNode& root,
                  port::RuntimeParameters& parsed,
                  bool& optional_malformed) {
    ReadOptionalNestedNumber(root,
                             "MOTION_ODOMETRY",
                             "ENCODER_TICKS_TO_METER",
                             parsed.motion_odometry.encoder_ticks_to_meter,
                             optional_malformed);
    const cv::FileNode ml = root["ML"];
    if (!ml.empty() && !ml.isMap()) {
        optional_malformed = true;
        return;
    }
    if (!ml.empty()) {
        ReadOptionalBool(ml, "ENABLED", parsed.ml.enabled, optional_malformed);
        const cv::FileNode roi = ml["ROI"];
        const cv::FileNode v9 = ml["V9"];
        const cv::FileNode tflite_identity = ml["TFLITE_IDENTITY"];
        const cv::FileNode mapping = ml["CLASS_MAPPING"];
        const cv::FileNode maneuver = ml["MANEUVER"];
        if ((!roi.empty() && !roi.isMap()) || (!v9.empty() && !v9.isMap()) ||
            (!tflite_identity.empty() && !tflite_identity.isMap()) ||
            (!mapping.empty() && !mapping.isMap()) ||
            (!maneuver.empty() && !maneuver.isMap())) {
            optional_malformed = true;
            return;
        }
        if (!roi.empty()) {
            ReadOptionalNumber(roi, "SEARCH_FORWARD_MIN_M", parsed.ml.roi.search_forward_min_m, optional_malformed);
            ReadOptionalNumber(roi, "SEARCH_FORWARD_MAX_M", parsed.ml.roi.search_forward_max_m, optional_malformed);
            ReadOptionalNumber(roi, "SEARCH_LATERAL_LIMIT_M", parsed.ml.roi.search_lateral_limit_m, optional_malformed);
            ReadOptionalNumber(roi, "GRID_FORWARD_STEP_M", parsed.ml.roi.grid_forward_step_m, optional_malformed);
            ReadOptionalNumber(roi, "GRID_LATERAL_STEP_M", parsed.ml.roi.grid_lateral_step_m, optional_malformed);
            ReadOptionalInt(roi, "RED_Y_MIN", parsed.ml.roi.red_y_min, optional_malformed);
            ReadOptionalInt(roi, "RED_Y_MAX", parsed.ml.roi.red_y_max, optional_malformed);
            ReadOptionalInt(roi, "RED_U_MIN", parsed.ml.roi.red_u_min, optional_malformed);
            ReadOptionalInt(roi, "RED_U_MAX", parsed.ml.roi.red_u_max, optional_malformed);
            ReadOptionalInt(roi, "RED_V_MIN", parsed.ml.roi.red_v_min, optional_malformed);
            ReadOptionalInt(roi, "RED_V_MAX", parsed.ml.roi.red_v_max, optional_malformed);
            ReadOptionalNumber(roi, "EXPECTED_LONG_EDGE_M", parsed.ml.roi.expected_long_edge_m, optional_malformed);
            ReadOptionalNumber(roi, "EXPECTED_SHORT_EDGE_M", parsed.ml.roi.expected_short_edge_m, optional_malformed);
            ReadOptionalNumber(roi, "CROP_LONG_OFFSET_M", parsed.ml.roi.crop_long_offset_m, optional_malformed);
            ReadOptionalNumber(roi, "CROP_FORWARD_OFFSET_M", parsed.ml.roi.crop_forward_offset_m, optional_malformed);
            ReadOptionalNumber(roi, "LONG_EDGE_TOLERANCE_M", parsed.ml.roi.long_edge_tolerance_m, optional_malformed);
            ReadOptionalNumber(roi, "SHORT_EDGE_TOLERANCE_M", parsed.ml.roi.short_edge_tolerance_m, optional_malformed);
            ReadOptionalNumber(roi, "MAX_LONG_EDGE_TO_LATERAL_RAD", parsed.ml.roi.max_long_edge_to_lateral_rad, optional_malformed);
            ReadOptionalInt(roi, "MIN_COMPONENT_CELLS", parsed.ml.roi.min_component_cells, optional_malformed);
            ReadOptionalNumber(roi, "MIN_RECTANGULARITY", parsed.ml.roi.min_rectangularity, optional_malformed);
            ReadOptionalNumber(roi, "MIN_RED_FILL_RATIO", parsed.ml.roi.min_red_fill_ratio, optional_malformed);
            ReadOptionalNumber(roi, "SCORE_SIZE_WEIGHT", parsed.ml.roi.score_size_weight, optional_malformed);
            ReadOptionalNumber(roi, "SCORE_RECTANGULARITY_WEIGHT", parsed.ml.roi.score_rectangularity_weight, optional_malformed);
            ReadOptionalNumber(roi, "SCORE_RED_FILL_WEIGHT", parsed.ml.roi.score_red_fill_weight, optional_malformed);
            ReadOptionalNumber(roi, "SCORE_ORIENTATION_WEIGHT", parsed.ml.roi.score_orientation_weight, optional_malformed);
        }
        if (!v9.empty()) {
            ReadOptionalInt(v9, "MIN_MARGIN", parsed.ml.v9.min_margin, optional_malformed);
            ReadOptionalInt(v9, "MAX_BEST_DISTANCE", parsed.ml.v9.max_best_distance, optional_malformed);
            ReadOptionalInt(v9, "CONFIRM_FRAMES", parsed.ml.v9.confirm_frames, optional_malformed);
        }
        if (!tflite_identity.empty()) {
            ReadOptionalInt(tflite_identity, "MIN_MARGIN", parsed.ml.tflite_identity.min_margin,
                            optional_malformed);
            ReadOptionalInt(tflite_identity, "MAX_BEST_DISTANCE",
                            parsed.ml.tflite_identity.max_best_distance, optional_malformed);
            ReadOptionalInt(tflite_identity, "CONFIRM_FRAMES",
                            parsed.ml.tflite_identity.confirm_frames, optional_malformed);
        }
        if (!mapping.empty()) {
            if (!mapping["CLASS_0_ACTION"].empty() &&
                !ReadStringValue(mapping["CLASS_0_ACTION"], parsed.ml.class_mapping.class_0_action)) optional_malformed = true;
            if (!mapping["CLASS_1_ACTION"].empty() &&
                !ReadStringValue(mapping["CLASS_1_ACTION"], parsed.ml.class_mapping.class_1_action)) optional_malformed = true;
            if (!mapping["CLASS_2_ACTION"].empty() &&
                !ReadStringValue(mapping["CLASS_2_ACTION"], parsed.ml.class_mapping.class_2_action)) optional_malformed = true;
        }
        if (!maneuver.empty()) {
            ReadOptionalBool(maneuver, "ENABLED", parsed.ml.maneuver.enabled, optional_malformed);
            ReadOptionalNumber(maneuver, "SPEED_TARGET", parsed.ml.maneuver.speed_target, optional_malformed);
            ReadOptionalInt(maneuver, "MIN_BOUNDARY_SAMPLES", parsed.ml.maneuver.min_boundary_samples, optional_malformed);
            ReadOptionalNumber(maneuver, "PATH_OUTWARD_OFFSET_M", parsed.ml.maneuver.path_outward_offset_m, optional_malformed);
            ReadOptionalNumber(maneuver, "EXIT_FORWARD_M", parsed.ml.maneuver.exit_forward_m, optional_malformed);
            ReadOptionalInt(maneuver, "MAX_DURATION_MS", parsed.ml.maneuver.max_duration_ms, optional_malformed);
            ReadOptionalInt(maneuver, "MAX_INTEGRATION_GAP_MS", parsed.ml.maneuver.max_integration_gap_ms, optional_malformed);
            ReadOptionalInt(maneuver, "COOLDOWN_MS", parsed.ml.maneuver.cooldown_ms, optional_malformed);
        }
    }
    if (!port::ValidateMlParameters(parsed.ml, parsed.motion_odometry)) {
        optional_malformed = true;
    }
}

void ReadBevProjectorParams(const cv::FileNode& root,
                            port::RuntimeParameters& parsed,
                            bool& optional_malformed) {
    ReadOptionalNestedBool(root, "BEV_PROJECTOR", "VALID", parsed.bev_projector.valid, optional_malformed);
    ReadOptionalNestedInt(
        root, "BEV_PROJECTOR", "DEBUG_GRID_WIDTH", parsed.bev_projector.debug_grid_width, optional_malformed);
    ReadOptionalNestedInt(
        root, "BEV_PROJECTOR", "DEBUG_GRID_HEIGHT", parsed.bev_projector.debug_grid_height, optional_malformed);
    ReadOptionalNestedString(
        root, "BEV_PROJECTOR", "PROJECTOR_ID", parsed.bev_projector.projector_id, optional_malformed);
    ReadOptionalNestedString(
        root, "BEV_PROJECTOR", "PROJECTOR_HASH", parsed.bev_projector.projector_hash, optional_malformed);
    for (int index = 0; index < static_cast<int>(port::kBevCalibrationPointCount); ++index) {
        const std::size_t point_index = static_cast<std::size_t>(index);
        ReadOptionalNestedNumber(root,
                                 "BEV_PROJECTOR",
                                 "SOURCE_ROW_" + std::to_string(index),
                                 parsed.bev_projector.source_points[point_index].row_px,
                                 optional_malformed);
        ReadOptionalNestedNumber(root,
                                 "BEV_PROJECTOR",
                                 "SOURCE_COL_" + std::to_string(index),
                                 parsed.bev_projector.source_points[point_index].col_px,
                                 optional_malformed);
        ReadOptionalNestedNumber(root,
                                 "BEV_PROJECTOR",
                                 "TARGET_FORWARD_" + std::to_string(index),
                                 parsed.bev_projector.target_points[point_index].forward_m,
                                 optional_malformed);
        ReadOptionalNestedNumber(root,
                                 "BEV_PROJECTOR",
                                 "TARGET_LATERAL_" + std::to_string(index),
                                 parsed.bev_projector.target_points[point_index].lateral_m,
                                 optional_malformed);
    }
}

void ReadBevGeometryParams(const cv::FileNode& root,
                           port::RuntimeParameters& parsed,
                           bool& optional_malformed) {
    for (int index = 0; index < static_cast<int>(port::kBevReferenceSampleCount); ++index) {
        ReadOptionalNestedNumber(root,
                                 "BEV_GEOMETRY",
                                 "FORWARD_SAMPLE_" + std::to_string(index),
                                 parsed.bev_geometry.forward_samples_m[static_cast<std::size_t>(index)],
                                 optional_malformed);
    }
    ReadOptionalNestedNumber(root,
                             "BEV_GEOMETRY",
                             "SEARCH_LATERAL_LIMIT_M",
                             parsed.bev_geometry.search_lateral_limit_m,
                             optional_malformed);
    ReadOptionalNestedNumber(root,
                             "BEV_GEOMETRY",
                             "LATERAL_STEP_M",
                             parsed.bev_geometry.lateral_step_m,
                             optional_malformed);
    ReadOptionalNestedNumber(root,
                             "BEV_GEOMETRY",
                             "NOMINAL_ROAD_HALF_WIDTH_M",
                             parsed.bev_geometry.nominal_road_half_width_m,
                             optional_malformed);
    ReadOptionalNestedNumber(root,
                             "BEV_GEOMETRY",
                             "BOUNDARY_TRACE_MAX_ADJACENT_DISTANCE_M",
                             parsed.bev_geometry.boundary_trace_max_adjacent_distance_m,
                             optional_malformed);
    ReadOptionalNestedInt(
        root, "BEV_GEOMETRY", "SPARSE_ROW_COUNT", parsed.bev_geometry.sparse_row_count, optional_malformed);
    if (!ValidateBEVGeometry(parsed.bev_geometry)) {
        optional_malformed = true;
    }
}

void ReadBevClassificationParams(const cv::FileNode& root,
                                 port::RuntimeParameters& parsed,
                                 bool& optional_malformed) {
    ReadOptionalNestedNumber(root,
                             "BEV_CLASSIFICATION",
                             "WHITE_CONFIDENCE_MIN",
                             parsed.bev_classification.white_confidence_min,
                             optional_malformed);
    ReadOptionalNestedNumber(root,
                             "BEV_CLASSIFICATION",
                             "UNKNOWN_CONFIDENCE_MIN",
                             parsed.bev_classification.unknown_confidence_min,
                             optional_malformed);
    ReadOptionalNestedInt(root,
                          "BEV_CLASSIFICATION",
                          "HOLD_LAST_MAX_CYCLES",
                          parsed.bev_classification.hold_last_max_cycles,
                          optional_malformed);
    if (!port::IsValidBEVClassificationParameters(parsed.bev_classification)) {
        optional_malformed = true;
    }
}

void ReadBevControlModelParams(const cv::FileNode& root,
                               port::RuntimeParameters& parsed,
                               bool& optional_malformed) {
    ReadOptionalNestedNumber(root,
                             "BEV_CONTROL_MODEL",
                             "LATERAL_OFFSET_TO_WHEEL_DELTA_GAIN",
                             parsed.bev_control_model.lateral_offset_to_wheel_delta_gain,
                             optional_malformed);
    ReadOptionalNestedNumber(root,
                             "BEV_CONTROL_MODEL",
                             "HEADING_ERROR_TO_WHEEL_DELTA_GAIN",
                             parsed.bev_control_model.heading_error_to_wheel_delta_gain,
                             optional_malformed);
    ReadOptionalNestedNumber(root,
                             "BEV_CONTROL_MODEL",
                             "CURVATURE_TO_WHEEL_DELTA_GAIN",
                             parsed.bev_control_model.curvature_to_wheel_delta_gain,
                             optional_malformed);
    ReadOptionalNestedInt(root,
                          "BEV_CONTROL_MODEL",
                          "MIN_LEADING_REFERENCE_SAMPLES",
                          parsed.bev_control_model.min_leading_reference_samples,
                          optional_malformed);
    ReadOptionalNestedInt(root,
                          "BEV_CONTROL_MODEL",
                          "TRACKING_FIT_MIN_SAMPLES",
                          parsed.bev_control_model.tracking_fit_min_samples,
                          optional_malformed);
    if (!ValidateBEVControlModel(parsed.bev_control_model)) {
        optional_malformed = true;
    }
}

void ReadBevElementParams(const cv::FileNode& root, port::RuntimeParameters& parsed, bool& optional_malformed) {
    ReadOptionalNestedInt(root,
                          "BEV_ELEMENT",
                          "CROSS_MIN_SAMPLEABLE_PER_ROW",
                          parsed.bev_element.cross_min_sampleable_per_row,
                          optional_malformed);
    ReadOptionalNestedInt(root,
                          "BEV_ELEMENT",
                          "CROSS_CONNECTIVITY_SAMPLE_INDEX",
                          parsed.bev_element.cross_connectivity_sample_index,
                          optional_malformed);
    ReadOptionalNestedNumber(root,
                             "BEV_ELEMENT",
                             "CROSS_BOUNDARY_EXPANSION_MIN_M",
                             parsed.bev_element.cross_boundary_expansion_min_m,
                             optional_malformed);
    ReadOptionalNestedBool(
        root, "BEV_ELEMENT", "CIRCLE_V2_ENABLED", parsed.bev_element.circle_v2_enabled, optional_malformed);
    ReadOptionalNestedNumber(root,
                             "BEV_ELEMENT",
                             "CIRCLE_V2_NORMAL_TRACE_START_YAW_DEG",
                             parsed.bev_element.circle_v2_normal_trace_start_yaw_deg,
                             optional_malformed);
    ReadOptionalNestedNumber(root, "BEV_ELEMENT", "CIRCLE_V2_EXIT_TRACE_START_YAW_DEG",
                             parsed.bev_element.circle_v2_exit_trace_start_yaw_deg,
                             optional_malformed);
    ReadOptionalNestedNumber(root, "BEV_ELEMENT", "CIRCLE_V2_CALM_FALLBACK_YAW_DEG",
                             parsed.bev_element.circle_v2_calm_fallback_yaw_deg,
                             optional_malformed);
    ReadOptionalNestedInt(root, "BEV_ELEMENT", "CIRCLE_V2_CALM_TRACE_MS",
                          parsed.bev_element.circle_v2_calm_trace_ms, optional_malformed);
    ReadOptionalNestedInt(root, "BEV_ELEMENT", "CIRCLE_V2_COOLDOWN_MS",
                          parsed.bev_element.circle_v2_cooldown_ms, optional_malformed);
    ReadOptionalNestedInt(root,
                          "BEV_ELEMENT",
                          "CIRCLE_V2_INNER_TRACE_STALL_TIMEOUT_MS",
                          parsed.bev_element.circle_v2_inner_trace_stall_timeout_ms,
                          optional_malformed);
    ReadOptionalNestedNumber(root,
                             "BEV_ELEMENT",
                             "CIRCLE_V2_INNER_TRACE_STALL_YAW_MIN_DEG",
                             parsed.bev_element.circle_v2_inner_trace_stall_yaw_min_deg,
                             optional_malformed);
    ReadOptionalNestedNumber(root,
                             "BEV_ELEMENT",
                             "CIRCLE_V2_INNER_TRACE_PATH_OFFSET_M",
                             parsed.bev_element.circle_v2_inner_trace_path_offset_m,
                             optional_malformed);
    ReadOptionalNestedNumber(root,
                             "BEV_ELEMENT",
                             "CIRCLE_V2_OPPOSITE_STRAIGHT_CONFIDENCE_MIN",
                             parsed.bev_element.circle_v2_opposite_straight_confidence_min,
                             optional_malformed);
    ReadOptionalNestedNumber(root, "BEV_ELEMENT", "CIRCLE_V2_MIN_SAMPLEABLE_WIDTH_M",
                             parsed.bev_element.circle_v2_min_sampleable_width_m, optional_malformed);
    ReadOptionalNestedNumber(root, "BEV_ELEMENT", "CIRCLE_V2_OPENING_FORWARD_MIN_M",
                             parsed.bev_element.circle_v2_opening_forward_min_m, optional_malformed);
    ReadOptionalNestedNumber(root, "BEV_ELEMENT", "CIRCLE_V2_OPENING_FORWARD_MAX_M",
                             parsed.bev_element.circle_v2_opening_forward_max_m, optional_malformed);
    ReadOptionalNestedNumber(root, "BEV_ELEMENT", "CIRCLE_V2_OPENING_DISTANCE_MIN_M",
                             parsed.bev_element.circle_v2_opening_distance_min_m, optional_malformed);
    ReadOptionalNestedNumber(root, "BEV_ELEMENT", "CIRCLE_V2_OPENING_CONFIRM_FORWARD_SPAN_M",
                             parsed.bev_element.circle_v2_opening_confirm_forward_span_m, optional_malformed);
    ReadOptionalNestedNumber(root, "BEV_ELEMENT", "CIRCLE_V2_ENTRY_FORWARD_MIN_M",
                             parsed.bev_element.circle_v2_entry_forward_min_m, optional_malformed);
    ReadOptionalNestedNumber(root, "BEV_ELEMENT", "CIRCLE_V2_ENTRY_FORWARD_MAX_M",
                             parsed.bev_element.circle_v2_entry_forward_max_m, optional_malformed);
    ReadOptionalNestedNumber(root, "BEV_ELEMENT", "CIRCLE_V2_INNER_GEOMETRY_FORWARD_MIN_M",
                             parsed.bev_element.circle_v2_inner_geometry_forward_min_m, optional_malformed);
    ReadOptionalNestedNumber(root, "BEV_ELEMENT", "CIRCLE_V2_INNER_GEOMETRY_FORWARD_MAX_M",
                             parsed.bev_element.circle_v2_inner_geometry_forward_max_m, optional_malformed);
    ReadOptionalNestedNumber(root, "BEV_ELEMENT", "CIRCLE_V2_EXIT_GEOMETRY_FORWARD_MIN_M",
                             parsed.bev_element.circle_v2_exit_geometry_forward_min_m, optional_malformed);
    ReadOptionalNestedNumber(root, "BEV_ELEMENT", "CIRCLE_V2_EXIT_GEOMETRY_FORWARD_MAX_M",
                             parsed.bev_element.circle_v2_exit_geometry_forward_max_m, optional_malformed);
    ReadOptionalNestedNumber(root, "BEV_ELEMENT", "CIRCLE_V2_EXIT_STRAIGHT_MAX_LATERAL_SPAN_M",
                             parsed.bev_element.circle_v2_exit_straight_max_lateral_span_m, optional_malformed);
    ReadOptionalNestedNumber(root, "BEV_ELEMENT", "CIRCLE_V2_EXIT_TANGENT_FIT_SPAN_M",
                             parsed.bev_element.circle_v2_exit_tangent_fit_span_m, optional_malformed);
    if (!ValidateBEVElement(parsed.bev_element)) {
        optional_malformed = true;
    }
}

void ReadReferenceTimeAlignmentParams(const cv::FileNode& root,
                                      port::RuntimeParameters& parsed,
                                      bool& optional_malformed) {
    ReadOptionalNestedBool(
        root, "REFERENCE_TIME_ALIGNMENT", "ENABLED", parsed.reference_time_alignment.enabled,
        optional_malformed);
    ReadOptionalNestedInt(
        root, "REFERENCE_TIME_ALIGNMENT", "MAX_AGE_MS", parsed.reference_time_alignment.max_age_ms,
        optional_malformed);
    ReadOptionalNestedInt(root,
                          "REFERENCE_TIME_ALIGNMENT",
                          "EFFECTIVE_DELAY_MS",
                          parsed.reference_time_alignment.effective_delay_ms,
                          optional_malformed);
    ReadOptionalNestedInt(root,
                          "REFERENCE_TIME_ALIGNMENT",
                          "FUTURE_PREDICTION_MAX_MS",
                          parsed.reference_time_alignment.future_prediction_max_ms,
                          optional_malformed);
    ReadOptionalNestedInt(root,
                          "REFERENCE_TIME_ALIGNMENT",
                          "MAX_INTEGRATION_GAP_MS",
                          parsed.reference_time_alignment.max_integration_gap_ms,
                          optional_malformed);
    ReadOptionalNestedInt(root,
                          "REFERENCE_TIME_ALIGNMENT",
                          "MIN_ALIGNED_SAMPLES",
                          parsed.reference_time_alignment.min_aligned_samples,
                          optional_malformed);
    ReadOptionalNestedBool(root,
                           "REFERENCE_TIME_ALIGNMENT",
                           "USE_ENCODER_FORWARD",
                           parsed.reference_time_alignment.use_encoder_forward,
                           optional_malformed);
    ReadOptionalNestedNumber(root,
                             "REFERENCE_TIME_ALIGNMENT",
                             "WHEEL_TRACK_M",
                             parsed.reference_time_alignment.wheel_track_m,
                             optional_malformed);
    ReadOptionalNestedBool(
        root, "REFERENCE_TIME_ALIGNMENT", "USE_IMU_YAW", parsed.reference_time_alignment.use_imu_yaw,
        optional_malformed);
    ReadOptionalNestedBool(root,
                           "REFERENCE_TIME_ALIGNMENT",
                           "USE_WHEEL_YAW_FALLBACK",
                           parsed.reference_time_alignment.use_wheel_yaw_fallback,
                           optional_malformed);
    ReadOptionalNestedBool(root,
                           "REFERENCE_TIME_ALIGNMENT",
                           "FUTURE_PREDICTION_ENABLED",
                           parsed.reference_time_alignment.future_prediction_enabled,
                           optional_malformed);
    ReadOptionalNestedBool(root,
                           "REFERENCE_TIME_ALIGNMENT",
                           "COMMAND_YAW_PREDICTION_ENABLED",
                           parsed.reference_time_alignment.command_yaw_prediction_enabled,
                           optional_malformed);
    ReadOptionalNestedNumber(root,
                             "REFERENCE_TIME_ALIGNMENT",
                             "TURN_OUTPUT_TO_YAW_RATE_GAIN",
                             parsed.reference_time_alignment.turn_output_to_yaw_rate_gain,
                             optional_malformed);
    ReadOptionalNestedNumber(root,
                             "REFERENCE_TIME_ALIGNMENT",
                             "ACTUATOR_YAW_TAU_MS",
                             parsed.reference_time_alignment.actuator_yaw_tau_ms,
                             optional_malformed);
    ReadOptionalNestedNumber(root,
                             "REFERENCE_TIME_ALIGNMENT",
                             "MAX_DELTA_FORWARD_M",
                             parsed.reference_time_alignment.max_delta_forward_m,
                             optional_malformed);
    ReadOptionalNestedNumber(root,
                             "REFERENCE_TIME_ALIGNMENT",
                             "MAX_DELTA_LATERAL_M",
                             parsed.reference_time_alignment.max_delta_lateral_m,
                             optional_malformed);
    ReadOptionalNestedNumber(root,
                             "REFERENCE_TIME_ALIGNMENT",
                             "MAX_DELTA_YAW_RAD",
                             parsed.reference_time_alignment.max_delta_yaw_rad,
                             optional_malformed);
    if (!ValidateReferenceTimeAlignment(parsed.reference_time_alignment,
                                        parsed.motion_odometry)) {
        optional_malformed = true;
    }
}

void ReadCameraSourceParams(const cv::FileNode& root, port::RuntimeParameters& parsed, bool& optional_malformed) {
    ReadOptionalNestedString(root, "CAMERA_SOURCE", "BACKEND", parsed.camera_source.backend, optional_malformed);
    ReadOptionalNestedString(root, "CAMERA_SOURCE", "DEVICE", parsed.camera_source.device, optional_malformed);
    ReadOptionalNestedInt(root, "CAMERA_SOURCE", "WIDTH", parsed.camera_source.width, optional_malformed);
    ReadOptionalNestedInt(root, "CAMERA_SOURCE", "HEIGHT", parsed.camera_source.height, optional_malformed);
    ReadOptionalNestedInt(root, "CAMERA_SOURCE", "FPS", parsed.camera_source.fps, optional_malformed);
    ReadOptionalNestedInt(
        root, "CAMERA_SOURCE", "BUFFER_COUNT", parsed.camera_source.buffer_count, optional_malformed);
    ReadOptionalNestedInt(
        root, "CAMERA_SOURCE", "POLL_TIMEOUT_MS", parsed.camera_source.poll_timeout_ms, optional_malformed);
    ReadOptionalNestedBool(root,
                           "CAMERA_SOURCE",
                           "DRAIN_READY_BUFFERS",
                           parsed.camera_source.drain_ready_buffers,
                           optional_malformed);
    if (!ValidateCameraSource(parsed.camera_source)) {
        optional_malformed = true;
    }
}

/**
 * 参数存储实现类 —— 实现 port::IParamStore 接口。
 * 从 JSON 配置文件中加载运行时参数和硬件配置。
 * 支持 JSON 注释剥离、文件读取、OpenCV FileStorage 解析。
 * 运行时参数文件缺失、解析失败或字段校验失败时拒绝启动。
 */
class ParamStore final : public port::IParamStore {
public:
    /**
     * 从 JSON 文件加载全部运行时参数。
     * 流程：读取文件 -> 剥离注释 -> 解析 JSON -> 提取必填字段和可选字段 -> 校验完整性。
     * 文件缺失、解析失败或字段校验失败时通过诊断输出原因并返回失败。
     * @param path JSON 配置文件路径
     * @param out 输出参数，加载后的运行时参数
     * @param diagnostics 诊断输出接收器
     * @return true 表示参数完整且有效并已写入 out；false 表示拒绝加载，out 不变
     */
    bool LoadRuntimeParameters(const std::string& path,
                               port::RuntimeParameters& out,
                               port::DiagnosticSink& diagnostics) override {
        std::string text;
        if (!ReadText(path, text)) {
            diagnostics.Emit({port::DiagnosticLevel::kFailSafe,
                              "params.missing",
                              "runtime parameter file missing; refusing startup: " + path,
                              port::NowMs()});
            return false;
        }

        cv::FileStorage json;
        if (!ParseJsonObject(text, json)) {
            diagnostics.Emit({port::DiagnosticLevel::kFailSafe,
                              "params.parse",
                              "runtime parameter parse failure (invalid JSON syntax or non-object root); "
                              "refusing startup: " + path,
                              port::NowMs()});
            return false;
        }

        const cv::FileNode root = json.root();
        port::RuntimeParameters parsed{};
        const bool required_ok = ReadRequiredRuntimeParams(root, parsed);
        bool optional_malformed = false;
        ReadControlParams(root, parsed, optional_malformed);
        ReadMediaParams(root, parsed, optional_malformed);
        ReadBevProjectorParams(root, parsed, optional_malformed);
        ReadBevGeometryParams(root, parsed, optional_malformed);
        ReadBevClassificationParams(root, parsed, optional_malformed);
        ReadBevControlModelParams(root, parsed, optional_malformed);
        ReadBevElementParams(root, parsed, optional_malformed);
        ReadCameraSourceParams(root, parsed, optional_malformed);
        ReadMlParams(root, parsed, optional_malformed);
        ReadReferenceTimeAlignmentParams(root, parsed, optional_malformed);
        // WheelTargetMixer consumes either the ordinary running target or the ML
        // maneuver target. Assistant overrides are independently capped at
        // RUNNING_SPEED_TARGET by AssistantProtocolDecoder. Validate the exact
        // production worst case before publishing any part of parsed to out.
        double maximum_wheel_base_target = parsed.running_speed_target;
        if (parsed.ml.maneuver.enabled) {
            const bool ml_speed_target_valid =
                std::isfinite(parsed.ml.maneuver.speed_target) &&
                parsed.ml.maneuver.speed_target > 0.0 &&
                parsed.ml.maneuver.speed_target <= control::kWheelSpeedTargetMax;
            optional_malformed |= !ml_speed_target_valid;
            if (ml_speed_target_valid) {
                maximum_wheel_base_target =
                    std::max(maximum_wheel_base_target, parsed.ml.maneuver.speed_target);
            }
        }
        optional_malformed |= !control::WheelTargetArithmeticIsFinite(
            maximum_wheel_base_target,
            parsed.raw_turn_output_limit,
            {parsed.wheel_turn_accel_delta_scale,
             parsed.wheel_turn_decel_delta_scale});
        if (!required_ok || optional_malformed) {
            diagnostics.Emit({port::DiagnosticLevel::kFailSafe,
                              "params.validation",
                              !required_ok
                                  ? "runtime parameter validation failed (missing, malformed, or "
                                    "out-of-range required field); refusing startup: " + path
                                  : "runtime parameter validation failed (malformed or out-of-range "
                                    "optional field); refusing startup: " + path,
                              port::NowMs()});
            return false;
        }

        parsed.loaded_from_defaults = false;
        parsed.parse_failure = false;
        out = parsed;
        diagnostics.Emit({port::DiagnosticLevel::kInfo,
                          "params.loaded",
                          "runtime parameters loaded from " + path,
                          port::NowMs()});
        return true;
    }

    /**
     * 从 JSON 文件加载硬件配置。
     * 按顺序解析 camera/imu/encoder/actuator/timer/persistence/display 共 7 个子系统配置块。
     * 任何子系统解析失败都将导致整体加载失败（fail-closed 策略）。
     * @param path JSON 硬件配置文件路径
     * @param out 输出参数，加载后的完整硬件配置
     * @param diagnostics 诊断输出接收器
     * @return true 表示所有子系统均解析成功，false 表示任一子系统解析失败
     */
    bool LoadHardwareProfile(const std::string& path,
                             port::HardwareProfile& out,
                             port::DiagnosticSink& diagnostics) override {
        std::string text;
        if (!ReadText(path, text)) {
            diagnostics.Emit({port::DiagnosticLevel::kFailSafe,
                              "profile.missing",
                              "hardware profile missing; refusing startup in fail-closed mode: " + path,
                              port::NowMs()});
            return false;
        }

        cv::FileStorage json;
        if (!ParseJsonObject(text, json)) {
            diagnostics.Emit({port::DiagnosticLevel::kFailSafe,
                              "profile.parse",
                              "hardware profile parse failure (invalid JSON object); refusing startup in fail-closed mode",
                              port::NowMs()});
            return false;
        }

        const cv::FileNode root = json.root();
        port::HardwareProfile parsed{};
        if (!ParseProfileBlock(root, "camera", parsed.camera)) {
            diagnostics.Emit({port::DiagnosticLevel::kFailSafe,
                              "profile.parse.camera",
                              ProfileBlockError("camera"),
                              port::NowMs()});
            return false;
        }
        if (!ParseProfileBlock(root, "imu", parsed.imu)) {
            diagnostics.Emit({port::DiagnosticLevel::kFailSafe,
                              "profile.parse.imu",
                              ProfileBlockError("imu"),
                              port::NowMs()});
            return false;
        }
        if (!ParseProfileBlock(root, "encoder", parsed.encoder)) {
            diagnostics.Emit({port::DiagnosticLevel::kFailSafe,
                              "profile.parse.encoder",
                              ProfileBlockError("encoder"),
                              port::NowMs()});
            return false;
        }
        if (!ParseProfileBlock(root, "actuator", parsed.actuator)) {
            diagnostics.Emit({port::DiagnosticLevel::kFailSafe,
                              "profile.parse.actuator",
                              ProfileBlockError("actuator"),
                              port::NowMs()});
            return false;
        }
        if (!ParseProfileBlock(root, "timer", parsed.timer)) {
            diagnostics.Emit({port::DiagnosticLevel::kFailSafe,
                              "profile.parse.timer",
                              ProfileBlockError("timer"),
                              port::NowMs()});
            return false;
        }
        if (!ParseProfileBlock(root, "persistence", parsed.persistence)) {
            diagnostics.Emit({port::DiagnosticLevel::kFailSafe,
                              "profile.parse.persistence",
                              ProfileBlockError("persistence"),
                              port::NowMs()});
            return false;
        }
        if (!ParseProfileBlock(root, "display", parsed.display)) {
            diagnostics.Emit({port::DiagnosticLevel::kFailSafe,
                              "profile.parse.display",
                              ProfileBlockError("display"),
                              port::NowMs()});
            return false;
        }

        out = parsed;
        diagnostics.Emit({port::DiagnosticLevel::kInfo,
                          "profile.loaded",
                          "hardware profile loaded from " + path,
                          port::NowMs()});
        return true;
    }

};

}  // namespace

/**
 * 创建参数存储实例（工厂函数）。
 * @return 指向 IParamStore 接口的唯一指针
 */
std::unique_ptr<port::IParamStore> MakeParamStore() {
    return std::make_unique<ParamStore>();
}

}  // namespace ls2k::platform
