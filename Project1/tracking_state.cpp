#include "armor_finder.h"
#include "show_image.h"

bool ArmorFinder::stateTrackingTarget(cv::Mat& src) {
    auto pos = target_box.rect;
    if ((pos & cv::Rect2d(0, 0, src.rows, src.cols)) != pos) {
        target_box = ArmorBox();
        cout << "pos out" << endl;
        return false;
    }

    // 获取相较于追踪区域两倍长款的区域，用于重新搜索，获取灯条信息
    cv::Rect2d bigger_rect;
    bigger_rect = getDetectROI(pos, 2, cv::Size(src.cols, src.rows));   //2是倍数
    cv::Mat roi = src(bigger_rect).clone();
    ArmorBox box;
    // 在区域内重新搜索。 
    if (findArmorBox(roi, box)) { // 如果成功获取目标，则利用搜索区域重新追踪
        target_box = box;
        target_box.rect.x += bigger_rect.x; //　添加roi偏移量
        target_box.rect.y += bigger_rect.y;
        for (auto& blob : target_box.light_blobs) {
            blob.rect.center.x += bigger_rect.x;
            blob.rect.center.y += bigger_rect.y;
        }
    }
    else {    // 如果没有成功搜索目标，则判断跟丢。
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
