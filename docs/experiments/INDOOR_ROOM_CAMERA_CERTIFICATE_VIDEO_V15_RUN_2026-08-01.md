# 2026-08-01 Gazebo V15 实际前视相机录制与障碍证书演示

## Material Passport

- Origin Skill: experiment-agent
- Origin Mode: run
- Origin Date: 2026-08-01
- Verification Status: UNVERIFIED
- Version Label: indoor_room_camera_certificate_video_v15
- Final empirical run status: `provisional_success`
- Final delivery video status: `completed`

`UNVERIFIED` 限制的是研究结论的外推范围；`provisional_success` 表示 V15
attempt2 的单 seed 服务器物理闭环和连续相机录制通过了当前经验门禁。最终
timestamp-resampled MP4 已完成、完整解码并通过独立时间轴与哈希复核；它仍不把
这次经验可视化提升为形式安全证明。

## 1. 结论

V15 已在服务器 Gazebo Classic 11 / ODE 室内双车道场景中完成一次实际
Gazebo/Ogre 前视 RGB CameraSensor 演示。seed `7320014` 的小车到达终点，
五个障碍全部激活，碰撞和道路越界均为 0。时间戳对齐的 measured-pose lifted
online distance CBF 持续参与 action projection；相机和证书叠加层只作为
observer，不向 controller 回传数据。

V15 相比 V14 增加了逐 camera callback 的无损文件队列和连续 PNG 录制。最终
attempt2 收到并归档 frame counter `1..410` 的全部 410 个 callback，没有缺帧、
无效帧或残留队列文件，录制状态为 `completed`。

最终渲染器先验证全部 410 个源 callback 的 PNG 哈希，再在 `0.1..39.2 s` 的固定
10 fps 仿真时间网格上选择最近的源时间戳；中点使用整数纳秒并确定性选择较早的
capture/counter。最终 MP4 为 392 帧、39.2 秒，最后一帧明确显示
`COMPLETED / goal_reached / 39.20 s`。

## 2. 冻结身份与服务器位置

| 项目 | 身份 |
|---|---|
| V15 config | `configs/experiments/dynamic_road_gazebo_classic_v15.yaml` |
| V15 config SHA-256 | `e25e603f43b99c6937b96ae8aafdac283b0e7c34f62f8cd57c9299b3ff69b29c` |
| 本地 source candidate | `source_indoor_camera_v15_queue_candidate_20260801T024712.tar.gz` |
| Source candidate SHA-256 | `caff5554d9f4af1e24363fc4bfe2ca08967fdaabdc75947c78037e8d440f5909` |
| Server source | `/data/users/csynth/cbf_rover_runs/source_indoor_camera_v15_queue_candidate_caff5554d9f4_r2` |
| Server build | `/data/users/csynth/cbf_rover_runs/build_indoor_camera_v15_queue_candidate_caff5554d9f4` |
| Server result root | `/data/users/csynth/cbf_rover_runs/results_indoor_camera_v15_queue_candidate_caff5554d9f4` |
| Accepted episode | `/data/users/csynth/cbf_rover_runs/results_indoor_camera_v15_queue_candidate_caff5554d9f4/episode_seed_7320014_attempt2` |
| Timestamp renderer source archive | `source_indoor_camera_v15_timestamp_candidate_20260801T031002.tar.gz` |
| Timestamp renderer source SHA-256 | `8b47ff67f2298a4e0711fe6cafb4f7be2a10008983c2652859d99142a12ecc14` |
| Timestamp renderer server source | `/data/users/csynth/cbf_rover_runs/source_indoor_camera_v15_timestamp_candidate_8b47ff67f229` |
| Final delivery root | `/data/users/csynth/cbf_rover_runs/results_indoor_camera_v15_timestamp_candidate_8b47ff67f229/delivery_seed_7320014_attempt1` |

服务器 source 目录末尾的 `_r2` 是本次冻结源码的实际路径；不得省略或把它与
build/result 目录名混写。

## 3. 服务器命令

工作目录：

```text
/data/users/csynth/cbf_rover_runs/source_indoor_camera_v15_queue_candidate_caff5554d9f4_r2
```

正式 episode 命令：

```bash
GAZEBO_PLUGIN_PATH=/data/users/csynth/cbf_rover_runs/build_indoor_camera_v15_queue_candidate_caff5554d9f4 \
PYTHONPATH=src timeout 600s \
/data/users/csynth/cbf_rover_runs/results_20260729T184104Z_4525410e0fbc/environment/venv/bin/python \
  -m cbf_rover.entrypoints.research \
  simulate-gazebo-classic-road-cbf-live \
  --config configs/experiments/dynamic_road_gazebo_classic_v15.yaml \
  --bridge-binary \
  /data/users/csynth/cbf_rover_runs/build_indoor_camera_v15_queue_candidate_caff5554d9f4/cbf_rover_gazebo_classic_bridge \
  --output-dir \
  /data/users/csynth/cbf_rover_runs/results_indoor_camera_v15_queue_candidate_caff5554d9f4/episode_seed_7320014_attempt2 \
  --seed 7320014 --no-gif
```

命令退出码为 0，run manifest 状态为 `provisional_success`，termination 为
`goal_reached`。独立 replay 未在本片执行。

最终视频重渲染工作目录：

```text
/data/users/csynth/cbf_rover_runs/source_indoor_camera_v15_timestamp_candidate_8b47ff67f229
```

等价的冻结重渲染命令如下；正式执行外层使用 `/usr/bin/time -v` 记录资源，并在
Python 中读取返回 manifest、仅当 `status == completed` 时退出 0：

```bash
PYTHONPATH=src timeout 600s \
/data/users/csynth/cbf_rover_runs/results_20260729T184104Z_4525410e0fbc/environment/venv/bin/python \
  -c 'from cbf_rover.adapters.gazebo.camera_video import render_camera_certificate_video; render_camera_certificate_video(recording_manifest_path=__import__("sys").argv[1], output_dir=__import__("sys").argv[2], terminal_reason="goal_reached", terminal_simulated_time_s=39.2)' \
  /data/users/csynth/cbf_rover_runs/results_indoor_camera_v15_queue_candidate_caff5554d9f4/episode_seed_7320014_attempt2/live_display/camera_recording/recording_manifest.json \
  /data/users/csynth/cbf_rover_runs/results_indoor_camera_v15_timestamp_candidate_8b47ff67f229/delivery_seed_7320014_attempt1/camera_video
```

重渲染退出码为 0，墙钟耗时 `2:31.49`，最大常驻内存 `1086036 KiB`，swap 为 0。
服务器 Ruff 门禁通过，相关单元测试为 `31 passed in 11.43 s`。第一次真实编码
smoke 复用了缺少新字段 `run_simulated_duration_s` 的旧 fixture，因此在编码前按
fail-closed 合同以 `KeyError` 失败并保留；补齐冻结时长的新 fixture 随后通过全部
GStreamer 编码、探针、完整解码和终帧门禁。没有通过修改门槛或删除失败目录来
取得最终结果。

运行时 dashboard 只绑定服务器 loopback `127.0.0.1:11571`。远程观察需要：

```bash
ssh -L 11571:127.0.0.1:11571 ai-server
```

## 4. 物理闭环结果

| 指标 | attempt2 结果 |
|---|---:|
| seed | `7320014` |
| termination | `goal_reached` |
| transitions / states | `784 / 785` |
| simulated duration | `39.2 s` |
| activated obstacles | `5 / 5` |
| collisions / road departures | `0 / 0` |
| minimum lifted certificate barrier | `0.17925236430546335 m` |
| minimum lifted physical clearance | `0.2292523643054634 m` |
| minimum CBF constraint residual | `-8.326672684688674e-17 m/s` |
| filter intervention steps | `130` |
| maximum position tracking residual | `0.012691871852315979 m` |
| maximum heading tracking residual | `0.046286371880428945 rad` |

数值只描述这一次 Gazebo ODE 轨迹。最小 CBF residual 的负值处于浮点舍入量级，
并通过配置中的 `-1.0e-9` 经验门限；不能据此推出形式不变性或概率安全结论。

关键物理证据：

| 制品 | SHA-256 |
|---|---|
| `run_manifest.json` | `b9c9d4116105ff8d85aebba78d8e9048d17d1b6fe406f2789855fece0b4c3ecb` |
| `live_trace.jsonl` | `ee4522fcd3e7c6d71f456e7ebb8d3a29d474302ecbc1bddf3a3293c54febc9e2` |

## 5. 连续相机录制证据

V15 camera plugin 在保留原子 latest-frame 预览的同时，将每次 callback 发布到
独立队列文件。Python observer 逐个消费队列帧，校验 BMP 与嵌入式 Gazebo
时间戳，绑定最近的 `0.05 s` measured state，再归档 PNG 与 JSONL 索引。

| 指标 | attempt2 结果 | 冻结门限 |
|---|---:|---:|
| camera callbacks / archived frames | `410 / 410` | all callbacks |
| first / last frame counter | `1 / 410` | contiguous |
| missing frames | `0` | `0` |
| invalid frames | `0` | `0` |
| capture simulated-time span | `39.059 s` | diagnostic |
| capture coverage | `0.9964030612244896` | `>= 0.98` |
| median source-frame gap | `0.1 s` | diagnostic |
| maximum source-frame gap | `0.393 s` | `<= 0.5 s` |
| maximum camera/state alignment error | `0.025 s` | `<= 0.026 s` |
| queue remaining / temporary files | `0 / 0` | `0 / 0` |
| recording status | `completed` | `completed` |

录制制品：

| 制品 | SHA-256 |
|---|---|
| `live_display/camera_recording/recording_manifest.json` | `0a49bdd5f865c2a846e8af02e31bf0ba8776e74c4e746dce97e392a059b2aaa2` |
| `live_display/camera_recording/frame_index.jsonl` | `b580bf591cfe85319b7782a38fa6345e0c810e640e700be37f96ae71028adf02` |

这些值证明的是虚拟 CameraSensor callback 的连续归档与时间对齐，不是实物相机
采集，也不表示图像参与障碍检测或控制。

## 6. 保留的首次失败

首次完整 V15 尝试使用旧 source candidate：

```text
source archive SHA-256:
9c775c83143e9afe52a29886b9453e4253109ad363474368d140ff536f1d5a96

server source:
/data/users/csynth/cbf_rover_runs/source_indoor_camera_v15_candidate_9c775c83143e

server result:
/data/users/csynth/cbf_rover_runs/results_indoor_camera_v15_candidate_9c775c83143e/episode_seed_7320014_attempt1
```

该版本通过轮询 single latest-BMP 录制相机帧；在下一 callback 覆盖 latest 文件
之前没有逐帧队列，完整运行漏掉 5 个 camera callback，因此失败并保留。后续
`caff5554...` candidate 将每个 callback 原子写入独立队列；attempt2 的
counter `1..410` 连续且 missing 为 0。失败结果没有被删除或覆盖。

## 7. 最终视频交付状态

V15 的目标输出合同是 camera-primary、map-secondary、telemetry-collapsed 的
1920x1080 H.264 MP4，证书叠加层使用 timestamp-aligned measured pose。首次中间
编码曾把 410 个 callback 逐个按 10 fps 编码成 `41.0 s`，与 `39.2 s` 仿真时长
不符，因此只保留为诊断制品，SHA-256 为
`ac381897a9e62907ffc5673f2f16f08880368a502e60fc69f72fa7940f6f8b91`。

最终交付从同一组冻结源图按仿真时间戳重采样，不做运动插值。392 个输出帧映射到
351 个唯一源 counter；41 个输出帧复用最近源图，59 个非均匀或重复 callback 不进入
固定 10 fps 时间轴。最大源图映射误差为 `0.155 s`，source counter 映射单调不减。
O1--O5 的相机内证书分别在 `40 / 52 / 62 / 42 / 50` 个输出帧中可见。

| 最终视频字段 | 完成值 |
|---|---|
| delivery status | `completed` |
| MP4 file | `camera_video/gazebo_front_camera_certificate_1x.mp4` |
| MP4 bytes | `22742982` |
| MP4 SHA-256 | `503b00b50bd027d96d8ae83ba235aadfd51468c0782e37717803e51a4fd08f41` |
| encoded duration | `39.2 s` |
| encoded frame count | `392` |
| nominal frame rate | `10 fps` |
| codec / profile | `H.264 AVC, constrained-baseline, level 4` |
| pixel / color | `4:2:0 (I420 encoder input), BT.709, progressive` |
| video manifest SHA-256 | `44dd941e10b242120cb49b7d305951897664018eb876396312f195380f360f0c` |
| frame-index manifest | `camera_video/rendered_frame_index.jsonl` |
| frame-index SHA-256 | `96c0cc8fb1b09d4fa43f89f48b9aec636920077b3c6220792c8aef38caa24f2c` |
| encoder command log | `camera_video/logs/gst_launch.command.json` |
| encoder command log SHA-256 | `30dd59ba1dd4b332e53b7b75561f82a5b27c350af207c4d4247c6ce061f6716c` |
| media probe output | `camera_video/logs/gst_discoverer.stdout.log` |
| media probe SHA-256 | `de51e8f105847daea01c2319dd7fa23425a9f0c1fd4d7366950ea8beb8aeef52` |
| decode validation log | `camera_video/logs/gst_decode_validation.stderr.log` |
| decode validation log SHA-256 | `e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855` |
| QA contact sheet | `camera_video/qa_review/qa_contact_sheet.png` |
| QA contact-sheet SHA-256 | `4d009a1e34d9121886e7f5e9aa42e0709bdd6c4e83d916ceaa8bfdd153cc1cfa` |
| independent validation SHA-256 | `864961aaa7c6c86e49abd234cd3a23d2ec6fa9bb8f1cf9a41d18c544cd7e8b8a` |
| visual QA acceptance SHA-256 | `cd52b3675c1ae3dbab7fbbc486dd522923b672d63a2119a87010cc0326479c2f` |
| delivery manifest SHA-256 | `d30dcd63a31e562be555c13177417cb1315daa92e5e77409198e10813b0c2308` |

`gst-discoverer-1.0` 报告 1920x1080、10/1 fps、BT.709、progressive 和精确
`39.200000000 s`；`qtdemux ! avdec_h264 ! videoconvert ! fakesink` 完整解码返回
0。独立 OpenCV 全量解码得到 392 帧，并逐一重算了 410 个源 PNG、392 个输出 PNG、
两个索引和 MP4 的哈希。16 宫格可视检查确认参数位于相机外侧、道路未被遮挡、
五个障碍的琥珀证书均出现，终帧显示完成状态。

## 8. 声明边界

- 画面来自实际运行的虚拟 Gazebo/Ogre RGB CameraSensor，不是生成的 top-down
  trace 图，也不是真实摄像头；
- 图像不参与障碍检测、controller 或 CBF，不能称为视觉感知安全闭环；
- action projection 使用 measured-pose lifted online robust distance CBF；
- 相机中的琥珀边界是该 online CBF 的 observer-only 几何投影，不是新综合的
  证书；
- 固定几何 q-SBC 仍为 `not_ready`，本次录制不能替代 q-SBC 概率证明；
- V13 的 tracking envelope 仍是 Gazebo 经验包络，不是形式跟踪误差界；
- 本结果只有 seed `7320014`，属于单 seed 经验 Gazebo ODE 结果；
- 不声明随机几何泛化、独立 replay、环形跑道、ROS2、RViz、真实摄像头或实车
  安全。
