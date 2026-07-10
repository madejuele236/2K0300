#include "zf_common_headfile.hpp"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#ifndef PARAM_FILE_NAME
#define PARAM_FILE_NAME "params_config.txt"
#endif
//三个决策 0-在直行 1-左转 2-右转
// 全局变量 Flash，用于存储当前的 PID 参数
// 在嵌入式系统中，通常用 Flash 命名表示掉电非易失性存储的数据
FlashInformation Flash = {
    .kp = 1.0f,        // 写成小数
    .ki = 2.0f,
    .kd = 3.1415926f,
    .small_err = 15,
    .picture_err =30,
    .forward = 24,
    .debug_rgb_r_min = 120,
    .debug_rgb_rb_diff = 35,
    .debug_rgb_rg_diff = 35
};

/**
 * @brief 解析配置文件中的单行数据，提取指定键值的浮点数
 * 
 * 该函数尝试从输入的一行字符串中解析出 "key=value" 格式的数据。
 * 如果解析成功且 key 与目标 target_key 匹配，则将 value 写入 out_value 并返回 1。
 * 
 * @param line 输入的一行字符串（来自配置文件）
 * @param target_key 需要查找的目标键名（如 "kp", "ki", "kd"）
 * @param out_value 输出指针，用于存储解析到的浮点数值
 * @return int 匹配并解析成功返回 1，否则返回 0
 */
static int parse_line(const char* line, const char* target_key, float* out_value) {
    char key_buf[32] = {0};
    float val_buf = 0.0f;
    
    // 使用 sscanf 解析 "字符串=浮点数" 格式
    // %31[^=] 表示读取最多31个非 '=' 字符作为 key
    if (sscanf(line, "%31[^=]=%f", key_buf, &val_buf) == 2) {
        char* p = key_buf;
        // 去除 key 末尾可能存在的回车换行符
        while(*p) {
            if(*p == '\r' || *p == '\n') *p = 0;
            p++;
        }
        
        // 比较解析出的 key 是否为目标 key
        if (strcmp(key_buf, target_key) == 0) {
            *out_value = val_buf;
            return 1; 
        }
    }
    return 0;
}

/**
 * @brief 参数初始化函数，从文件加载 PID 参数
 * 
 * 该函数在系统启动时调用。它尝试打开配置文件 "params_config.txt"，
 * 逐行读取并解析 kp, ki, kd 的值。
 * - 如果文件中存在有效参数，则更新全局变量 Flash 并打印加载信息。
 * - 如果文件不存在、为空或未找到任何参数，则使用默认值（即全局变量初始值），
 *   并自动调用 Param_SaveAll() 将默认值写入文件，以便下次启动读取。
 */
void Param_Init(void) {
    printf("[Param] Initializing...\r\n");
    
    
    zf_driver_file_string file(PARAM_FILE_NAME, "r");
    char line_buf[64] = {0};
    int found_count = 0;
    float temp_val = 0.0f;
    
    // 循环读取文件的每一行，read_string 返回 0 表示读取成功
    while (file.read_string(line_buf) == 0) {
        // 尝试解析当前行是否为 kp, ki 或 kd
        if (parse_line(line_buf, "kp", &temp_val)) {
            Flash.kp = temp_val; 
            found_count++;
        }
        else if (parse_line(line_buf, "ki", &temp_val)) {
            Flash.ki = temp_val;
            found_count++;
        }
        else if (parse_line(line_buf, "kd", &temp_val)) {
            Flash.kd = temp_val;
            found_count++;
        }
        else if (parse_line(line_buf, "small_err", &temp_val)) {
            Flash.small_err = temp_val;
            found_count++;
        }
        else if (parse_line(line_buf, "picture_err", &temp_val)) {
            Flash.picture_err = temp_val;
            found_count++;
        }
        else if (parse_line(line_buf, "forward", &temp_val)) {
            Flash.forward = temp_val;
            found_count++;
        }
        else if (parse_line(line_buf, "debug_rgb_r_min", &temp_val)) {
            Flash.debug_rgb_r_min = temp_val;
            found_count++;
        }
        else if (parse_line(line_buf, "debug_rgb_rb_diff", &temp_val)) {
            Flash.debug_rgb_rb_diff = temp_val;
            found_count++;
        }
        else if (parse_line(line_buf, "debug_rgb_rg_diff", &temp_val)) {
            Flash.debug_rgb_rg_diff = temp_val;
            found_count++;
        }
        
        
        memset(line_buf, 0, sizeof(line_buf));
    }
    
    if (found_count > 0) {
        printf("[Param] Loaded: kp=%.2f, ki=%.2f, kd=%.4f\r\n", Flash.kp, Flash.ki, Flash.kd);
    } else {
        printf("[Param] File not found or empty. Using defaults.\r\n");
        // 文件不存在或为空，保存默认值到文件，确保下次启动能读取到
        Param_SaveAll();
    }
}

/**
 * @brief 保存单个参数到配置文件
 * 
 * 该函数用于动态更新某个特定的参数（如只修改 kp）。
 * 逻辑如下：
 * 1. 读取原文件所有内容到内存缓冲区。
 * 2. 遍历每一行，如果该行是要修改的 key，则用新值替换；否则保留原行。
 * 3. 如果原文件中不存在该 key，则在文件末尾追加新行。
 * 4. 将处理后的完整内容写回文件。
 * 
 * @param key 要保存的参数键名（"kp", "ki", 或 "kd"）
 * @param value 要保存的新浮点数值
 */
void Param_SaveSingle(const char* key, float value) {
    char new_line[64] = {0};
    sprintf(new_line, "%s=%.4f\r\n", key, value);
    
    char full_content[1024] = {0};
    char line[64] = {0};
    int updated = 0;
    float temp_check = 0.0f;
    
    // 以只读模式打开文件，准备读取旧内容
    zf_driver_file_string f_in(PARAM_FILE_NAME, "r");
    while(f_in.read_string(line) == 0) {
        // 检查当前行是否是目标 key
        if (parse_line(line, key, &temp_check)) {
            // 如果是，写入新的 key=value 行，标记已更新
            strcat(full_content, new_line);
            updated = 1;
        } else {
            // 如果不是，保留原行
            strcat(full_content, line);
            // 确保每行末尾有换行符，防止拼接后格式混乱
            int len = strlen(full_content);
            if(len > 0 && full_content[len-1] != '\n') {
                strcat(full_content, "\r\n");
            }
        }
        memset(line, 0, sizeof(line));
    }
    
    // 如果文件中原本没有这个 key，则追加到末尾
    if (!updated) {
        strcat(full_content, new_line);
    }
    
    // 以写入模式打开文件，将合并后的内容写回
    zf_driver_file_string f_out(PARAM_FILE_NAME, "w");
    f_out.write_string(full_content);
    
    printf("[Param] Saved %s = %.4f\r\n", key, value);
}

/**
 * @brief 保存所有参数到配置文件
 * 
 * 该函数将全局变量 Flash 中的 kp, ki, kd 当前值
 * 格式化为文本，直接覆盖写入配置文件。
 * 通常在初始化失败或需要重置所有参数时调用。
 */
void Param_SaveAll(void) {
    char content[256] = {0};
    
    sprintf(content, 
            "kp=%.4f\r\n"
            "ki=%.4f\r\n"
            "kd=%.4f\r\n"
            "small_err=%d\r\n"
            "picture_err=%d\r\n"
            "forward=%d\r\n"
            "debug_rgb_r_min=%d\r\n"
            "debug_rgb_rb_diff=%d\r\n"
            "debug_rgb_rg_diff=%d\r\n",
            Flash.kp,
            Flash.ki,
            Flash.kd,
            Flash.small_err,
            Flash.picture_err,
            Flash.forward,
            Flash.debug_rgb_r_min,
            Flash.debug_rgb_rb_diff,
            Flash.debug_rgb_rg_diff);
            
    // 写入文件
    zf_driver_file_string file(PARAM_FILE_NAME, "w");
    file.write_string(content);
    printf("[Param] All params saved.\r\n");
}