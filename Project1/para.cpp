#include "para.h"

float armor_angle = 0;  //װ�װ�Ƕ�
//װ�װ��ٶȼ�¼
float armor_speed = 0;
float last_armor_speed = 0;
float lastest_armor_speed = 0;
float lastests_armor_speed = 0;
float lastestss_armor_speed = 0;
bool find_armor = 0;  //�Ƿ��ҵ�װ�װ�

/**
 * ��������
 * @param int argc
 * @param char argv
 * @param string name ������
 * @param string value ��������
 * @return bool
 */
bool parseParameter(size_t argc, char* argv[], string name, int word) {
    string  s_word = "";
    for (size_t i = 0; i < argc; i++) {
        // '-'��'--'��ͷΪ������ʶ
        if ("-" + name == argv[i] || "--" + name == argv[i]) {
            // ������û�����ݣ��򷵻ؿ��ַ�
            if (argc < i + 1) {
                return true;
            }
            // �Ӳ���λ�ĺ�һλ��ʼȡֵ
            for (size_t j = i + 1; j < argc; j++) {
                string curr = argv[j];
                // ֱ������'-'��ͷ��ֹͣ
                if ("-" == curr.substr(0, 1)) {
                    return true;
                }
                // ƴ�ӿո�
                if (j != i + 1)
                {
                    s_word += " ";
                }
                // ƴ�ӵ�ǰ����
                s_word += argv[j];
                word = atoi(s_word.c_str());
            }
            return true;
        }
    }
    return false;
}