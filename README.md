# armor_predict
本项目来源于国防科技大学军临战队视觉组2022年装甲板击打跟踪算法开源内容的装甲板预测部分。项目基于Windows平台使用VS开发，主要用于Robomaster比赛中测试各机动目标模型对装甲板预测的效果以及相关参数调试。
### 功能实现
* 基于特征匹配的装甲板识别功能
* 基于Singer模型和当前统计模型的装甲板X轴像素坐标卡尔曼预测
* 模拟电控端延时预测的延时装甲板位置预测
* 卡尔曼滤波收敛判定
* 实时显示预测坐标和速度曲线
* 保存预测参数并自动调用gnuplot进行图线绘制<br/>
[演示视频](https://www.bilibili.com/video/BV1B3411u7gh?spm_id_from=333.999.list.card_archive.click)
### 依赖环境
#### 必备环境
* [Opencv3.x](https://opencv.org/releases/)
* [Eigen3](https://gitlab.com/libeigen/eigen/-/releases/)
#### 推荐环境
* [gnuplot](https://blog.csdn.net/qq_41941875/article/details/115691482)
### 代码使用
#### 编译方法
1、 使用VS打开sln文件<br/>
2、 进入属性配置页，根据opencv和eigen的位置配置路径<br/>
3、 编译运行
#### 读写文件
1、 读取参数文件：Singer卡尔曼模型共包含五个参数：目标机动频率a、时间间隔T以及卡尔曼滤波三参数：P、Q、R。我们将这五个参数以空格区分按上述顺序放入read.txt下以方便调试，程序运行会读取该参数。<br/>
2、 写入结果文件：程序会保存以下五个参数至result.txt中：测量值、预测值、更新值、收敛的延时预测值、发散的延时预测值。当某帧延时预测值收敛时保存至文件的发散预测值为0，反之亦然。<br/>
3、 视频保存文件：打开视频保存功能后，会将运行效果保存至result.avi中
#### 调试接口
提供命令行参数调试接口：<br/>
| 参数名称 | 类型 | 含义 |
|---|---|---|
|enemy_color|uint8_t|敌方装甲板颜色（蓝色0；红色1）|
|save_video|bool|是否保存视频|
|show_armor_box|bool|是否显示单个装甲板图像|
|show_armor_boxes|bool|是否显示多个装甲板图像|
|show_light_blobs|bool|是否显示灯条处理过程图像|
### 数据分析
使用Matlab或Gnuplot绘制程序保存的数据图线如下，我们重点关注系统的延时预测点的趋势，即图片中的红色点和绿色点。可以看出，系统准确的提前一段时间预测到了装甲板的位置，且误差较大点（绿色点）成功筛选了出来。<br/>
![图片](predict.png)
### 联系我们
阳佳奇 微信：yjq1301350150
刘宇轩 微信：lyx2752256797
