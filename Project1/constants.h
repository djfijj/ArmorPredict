
#define ENEMY_RED 1
#define ENEMY_BLUE 0
//这些参数需要调节来减少误识别（有理论的调，别盲试）
#define LIGHTBLOB_ANGLE (20)        // 判断两个灯条的角度差临界值
#define LIGHTBLOB_HEIGHT (100)        // 判断两个灯条的高度差临界值
#define LIGHTBLOB_GAP_MAX (5)        // 判断两个灯条的间距临界值
#define LIGHTBLOB_GAP_MIN (0.6)      
#define LIGHTBLOB_RATIO_MAX (3)        // 判断两个灯条的长度比临界值
#define LIGHTBLOB_RATIO_MIN (0.4)
#define LIGHTBLOB_STAGGER (30)       // 判断两个灯条的错位度临界值

