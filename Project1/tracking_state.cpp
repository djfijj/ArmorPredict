#include "armor_finder.h"
#include "show_image.h"

bool ArmorFinder::stateTrackingTarget(cv::Mat& src) {
    auto pos = target_box.rect;
    if ((pos & cv::Rect2d(0, 0, src.rows, src.cols)) != pos) {
        target_box = ArmorBox();
        cout << "pos out" << endl;
        return false;
    }

    // ��ȡ�����׷�����������������������������������ȡ������Ϣ
    cv::Rect2d bigger_rect;
    bigger_rect = getDetectROI(pos, 2, cv::Size(src.cols, src.rows));   //2�Ǳ���
    cv::Mat roi = src(bigger_rect).clone();
    ArmorBox box;
    // ������������������ 
    if (findArmorBox(roi, box)) { // ����ɹ���ȡĿ�꣬������������������׷��
        target_box = box;
        target_box.rect.x += bigger_rect.x; //�����roiƫ����
        target_box.rect.y += bigger_rect.y;
        for (auto& blob : target_box.light_blobs) {
            blob.rect.center.x += bigger_rect.x;
            blob.rect.center.y += bigger_rect.y;
        }
    }
    else {    // ���û�гɹ�����Ŀ�꣬���жϸ�����
        target_box = ArmorBox();
        return false;
    }
    return true;
}

cv::Rect getDetectROI(const cv::Rect& pos, int scale, cv::Size src_size) {
    cv::Rect detect_rect;
    detect_rect = cv::Rect(pos.x - pos.width * (scale - 1) / 2.0, pos.y - pos.height * (scale - 1) / 2.0, \
        pos.width * scale, pos.height * scale);
    detect_rect &= cv::Rect(0, 0, src_size.width, src_size.height);
    return detect_rect;
}
