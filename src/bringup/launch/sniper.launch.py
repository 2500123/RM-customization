from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue
from pathlib import Path


def generate_launch_description():
    launch_path = Path(__file__).resolve()
    project_root = launch_path.parents[3]  # 源码运行时: .../Pacific_vision
    if project_root.name == 'bringup' and (project_root / 'share').exists():
        # 安装运行时: .../Pacific_vision/install/bringup/share/bringup/launch/sniper.launch.py
        # parents[3] 会是 .../install/bringup，这里回退到工作区根目录
        project_root = project_root.parents[1]
    
    debug_dump_dir = str(project_root / 'sniper_debug_imgs')  # 调试图片保存目录
    debug_dump_enable = False          # 调试开关：每N帧保存5个窗口画面
    debug_dump_every_n_frames = 1     # 调试保存间隔(帧)
    dump_save_raw = False              # 保存编码端 Raw 窗口
    dump_save_roi = True              # 保存编码端 ROI 窗口
    dump_save_static = False           # 保存编码端 Static 窗口
    dump_save_final = True            # 保存编码端 Final 窗口
    dump_save_decoder = True          # 保存解码端窗口

    # Launch 参数：用于在“带宽/规则约束”内调清晰度与稳定性。
    # - 更清晰（同带宽）通常靠：降低 output_fps、使用更慢的 x264_preset、降低噪声(曝光/增益)。
    # - 更稳（抗绿屏）通常靠：更短 GOP、更低码率峰值、更严格限速/丢弃策略。
    arg_exposure_time = DeclareLaunchArgument('exposure_time', default_value='12000.0')   # us
    arg_gain = DeclareLaunchArgument('gain', default_value='10.0')
    arg_encode_size = DeclareLaunchArgument('encode_size', default_value='300')
    arg_output_fps = DeclareLaunchArgument('output_fps', default_value='20')
    arg_gop_seconds = DeclareLaunchArgument('gop_seconds', default_value='0.5')
    arg_x264_preset = DeclareLaunchArgument('x264_preset', default_value='slower')  # x264 preset: auto/ultrafast/.../veryslow
    # 目标编码码率（单位：kbps）。默认 96kbps ~= 12kB/s * 8。
    arg_target_bitrate_kbps = DeclareLaunchArgument('target_bitrate_kbps', default_value='96')
    # 发送硬上限（单位：kB/s）。目标码率建议略低于硬上限，留出关键帧/VBV 波动余量。
    arg_bandwidth_limit_kbytes = DeclareLaunchArgument('bandwidth_limit_kbytes', default_value='14.0')
    arg_force_monochrome = DeclareLaunchArgument('force_monochrome', default_value='false')

    exposure_time = LaunchConfiguration('exposure_time')
    gain = LaunchConfiguration('gain')
    encode_size = LaunchConfiguration('encode_size')
    output_fps = LaunchConfiguration('output_fps')
    gop_seconds = LaunchConfiguration('gop_seconds')
    x264_preset = LaunchConfiguration('x264_preset')
    target_bitrate_kbps = LaunchConfiguration('target_bitrate_kbps')
    bandwidth_limit_kbytes = LaunchConfiguration('bandwidth_limit_kbytes')
    force_monochrome = LaunchConfiguration('force_monochrome')

    # Typed values for ROS parameters
    p_exposure_time = ParameterValue(exposure_time, value_type=float)
    p_gain = ParameterValue(gain, value_type=float)
    p_encode_size = ParameterValue(encode_size, value_type=int)
    p_output_fps = ParameterValue(output_fps, value_type=int)
    p_gop_seconds = ParameterValue(gop_seconds, value_type=float)
    p_target_bitrate_kbps = ParameterValue(target_bitrate_kbps, value_type=int)
    p_bandwidth_limit_kbytes = ParameterValue(bandwidth_limit_kbytes, value_type=float)
    p_force_monochrome = ParameterValue(force_monochrome, value_type=bool)


    # 编码端容器（相机 + 编码器，同进程零拷贝）
    encoder_container = ComposableNodeContainer(
        name='sniper_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='hik_camera',
                plugin='hik_camera::HikCameraNode',
                name='hik_camera',
                parameters=[
                    {'exposure_time': p_exposure_time},  # 曝光时间(us)
                    {'gain': p_gain}                      # 模拟增益
                ],
                extra_arguments=[{'use_intra_process_comms': True}]  # 启用进程内零拷贝
            ),
            ComposableNode(
                package='doorlock_sniper',
                plugin='doorlock_sniper::VideoEncoderNode',
                name='video_encoder',
                parameters=[
                    {'input_topic': '/image_raw'},                       # 输入图像话题
                    {'target_bitrate': p_target_bitrate_kbps},           # 目标编码码率(kbps)
                    {'x264_preset': x264_preset},                        # x264 preset: auto/ultrafast/.../veryslow
                    {'output_fps': p_output_fps},                        # 输出帧率
                    {'gop_seconds': p_gop_seconds},                      # GOP 时长(s). 短 GOP 更抗丢包/限速丢数据
                    {'low_bitrate_threshold_kbps': 200},                  # 提码率仍走“稳态”参数，避免切模式导致花屏
                    {'force_low_bitrate_mode': False},                    # 强制低码率参数(调试用)
                    # packet_size 固定为 280B (VideoPacket.msg payload 大小)，C++ 强制覆盖
                    {'enable_display': False},                           # 编码端调试显示 (用 PyQt viewer 替代)
                    {'debug_dump_enable': debug_dump_enable},            # 开启后每N帧保存编码端窗口画面
                    {'debug_dump_every_n_frames': debug_dump_every_n_frames},  # 编码端保存间隔(帧)
                    {'debug_dump_save_raw': dump_save_raw},              # 编码端 Raw 窗口保存开关
                    {'debug_dump_save_roi': dump_save_roi},              # 编码端 ROI 窗口保存开关
                    {'debug_dump_save_static': dump_save_static},        # 编码端 Static 窗口保存开关
                    {'debug_dump_save_final': dump_save_final},          # 编码端 Final 窗口保存开关
                    {'debug_dump_dir': debug_dump_dir},                  # 调试图片根目录
                    {'crop_size': 800},                                  # 中心裁剪ROI大小
                    {'output_size': p_encode_size},                      # 编码分辨率
                    {'static_simplify': True},                           # 静态区域简化
                    {'motion_threshold': 14},                            # 运动检测阈值
                    {'motion_erode_px': 1},                              # 运动掩码腐蚀像素 (与 C++ 默认一致)
                    {'motion_dilate_px': 2},                             # 运动掩码膨胀像素 (与 C++ 默认一致)
                    {'motion_trail_frames': 3},                          # 拖影历史帧数 (与 C++ 默认一致，避免内存占用过高)
                    {'trail_disable_motion_ratio': 0.30},                # 全局运动比例超阈值时临时禁用拖影显示
                    {'bg_update_alpha': 0.01},                           # 背景模型更新速度
                    {'bg_blur_sigma': 1.2},                              # 静态区模糊强度 (与 C++ 默认一致)
                    {'center_clear_size': 150},                          # 中心保护区尺寸(像素)
                    {'force_monochrome': p_force_monochrome},            # 强制全画面灰度
                    {'bandwidth_limit_kbytes': p_bandwidth_limit_kbytes},# 发送硬上限(kB/s)
                    {'bandwidth_window_s': 2.0},                         # 限速滑动窗口时长(s)
                    {'max_tx_delay_s': 1.0}                              # 发送队列最大允许时延(s)
                ],
                extra_arguments=[{'use_intra_process_comms': True}]      # 启用进程内零拷贝
            )
        ],
        output='screen',
    )

    # 解码端（Python 节点，独立进程）
    decoder_node = Node(
        package='doorlock_decoder',       # 修正：只保留这一个 package 参数
        executable='decoder_node',        # 对应 setup.py 中的 entry point
        name='video_decoder',
        parameters=[
            {'topic': '/video_stream'},      # 订阅的视频流话题
            {'display': False},              # 解码端显示 (用 PyQt viewer 替代)
            {'width': p_encode_size},        # 解码期望宽度
            {'height': p_encode_size},       # 解码期望高度
            {'display_scale': 2},            # 显示放大倍数(300->600)
            {'crosshair_offset_x': 0},       # 准心相对中心X偏移
            {'crosshair_offset_y': 0},       # 准心相对中心Y偏移
            {'crosshair_width': 0.5},          # 准心线宽(像素)
            {'debug_dump_enable': debug_dump_enable},            # 开启后每N帧保存解码窗口画面
            {'debug_dump_every_n_frames': debug_dump_every_n_frames},  # 解码端保存间隔(帧)
            {'debug_dump_save_decoder': dump_save_decoder},      # 解码端窗口保存开关
            {'debug_dump_dir': debug_dump_dir}                  # 调试图片根目录
        ],
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([
        arg_exposure_time,
        arg_gain,
        arg_encode_size,
        arg_output_fps,
        arg_gop_seconds,
        arg_x264_preset,
        arg_target_bitrate_kbps,
        arg_bandwidth_limit_kbytes,
        arg_force_monochrome,
        encoder_container,
        decoder_node,
    ])
