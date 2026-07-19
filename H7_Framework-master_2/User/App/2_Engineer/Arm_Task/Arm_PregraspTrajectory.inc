#ifndef H7_FRAMEWORK_ARM_PREGRASP_TRAJECTORY_DATA
#define H7_FRAMEWORK_ARM_PREGRASP_TRAJECTORY_DATA

/*
 * 预抓取轨迹占位文件。
 *
 * 当前六轴正方向和角度比例尚未逐轴验证，因此必须保持为0。
 * 完成标定并重新运行reach_planner后，使用
 * reach_pregrasp/export_pregrasp_firmware.py生成真实系数覆盖本文件。
 */
#define ARM_PREGRASP_CALIBRATION_VALID 0U

#endif /* H7_FRAMEWORK_ARM_PREGRASP_TRAJECTORY_DATA */
