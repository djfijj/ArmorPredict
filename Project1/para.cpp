#include "para.h"

float armor_angle = 0;  //装甲板角度
//装甲板速度记录
float armor_speed = 0;
float last_armor_speed = 0;
float lastest_armor_speed = 0;
float lastests_armor_speed = 0;
float lastestss_armor_speed = 0;
bool find_armor = 0;  //是否找到装甲板

/**
 * 解析参数
 * @param int argc
 * @param char argv
 * @param string name 参数名
 * @param string value 参数内容
 * @return bool
 */
bool parseParameter(size_t argc, char* argv[], string name, int word) {
    string  s_word = "";
    for (size_t i = 0; i < argc; i++) {
        // '-'或'--'开头为参数标识
        if ("-" + name == argv[i] || "--" + name == argv[i]) {
            // 参数后没有内容，则返回空字符
            if (argc < i + 1) {
                return true;
            }
            // 从参数位的后一位开始取值
            for (size_t j = i + 1; j < argc; j++) {
                string curr = argv[j];
                // 直到遇到'-'开头则停止
                if ("-" == curr.substr(0, 1)) {
                    return true;
                }
                // 拼接空格
                if (j != i + 1)
                {
                    s_word += " ";
                }
                // 拼接当前数据
                s_word += argv[j];
                word = atoi(s_word.c_str());
            }
            return true;
        }
    }
    return false;
}