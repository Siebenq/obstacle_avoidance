#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
视频和点云测试服务器
用于测试video_client ROS2包
"""

import socket
import struct
import time
import threading
import os
import sys
import argparse
import numpy as np

def create_test_pcd(filename='/tmp/test_cloud.pcd', num_points=1000):
    """
    创建测试PCD文件
    """
    # 生成随机点云数据（立方体形状）
    points = np.random.rand(num_points, 3) * 2.0 - 1.0  # [-1, 1] 范围
    
    with open(filename, 'w') as f:
        # PCD文件头
        f.write("# .PCD v0.7 - Point Cloud Data file format\n")
        f.write("VERSION 0.7\n")
        f.write("FIELDS x y z\n")
        f.write("SIZE 4 4 4\n")
        f.write("TYPE F F F\n")
        f.write("COUNT 1 1 1\n")
        f.write(f"WIDTH {num_points}\n")
        f.write("HEIGHT 1\n")
        f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
        f.write(f"POINTS {num_points}\n")
        f.write("DATA ascii\n")
        
        # 点数据
        for point in points:
            f.write(f"{point[0]} {point[1]} {point[2]}\n")
    
    print(f"✅ 创建测试PCD文件: {filename} ({num_points}点)")
    return filename

def pointcloud_server(port=5001, pcd_file=None, hz=10):
    """
    点云发送服务器
    """
    print(f"☁️  启动点云服务器 - 端口: {port}, 频率: {hz}Hz")
    
    # 如果没有指定PCD文件，创建测试文件
    if pcd_file is None or not os.path.exists(pcd_file):
        pcd_file = create_test_pcd()
    
    # 创建TCP socket
    server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_sock.bind(('0.0.0.0', port))
    server_sock.listen(1)
    
    print(f"⏳ 等待客户端连接 (端口 {port})...")
    
    while True:
        try:
            client_sock, client_addr = server_sock.accept()
            print(f"✅ 点云客户端已连接: {client_addr}")
            
            frame_count = 0
            
            while True:
                try:
                    # 读取PCD文件
                    with open(pcd_file, 'rb') as f:
                        pcd_data = f.read()
                    
                    # 发送大小（网络字节序，大端）
                    size = struct.pack('!I', len(pcd_data))
                    client_sock.sendall(size)
                    
                    # 发送数据
                    client_sock.sendall(pcd_data)
                    
                    frame_count += 1
                    if frame_count % 10 == 0:
                        print(f"☁️  已发送点云帧: {frame_count} (大小: {len(pcd_data)/1024:.2f}KB)")
                    
                    time.sleep(1.0 / hz)
                    
                except (ConnectionResetError, BrokenPipeError):
                    print(f"⚠️  客户端断开连接")
                    break
                except Exception as e:
                    print(f"❌ 发送错误: {e}")
                    break
            
            client_sock.close()
            print(f"⏳ 等待新的客户端连接...")
            
        except KeyboardInterrupt:
            break
        except Exception as e:
            print(f"❌ 服务器错误: {e}")
            time.sleep(1)
    
    server_sock.close()
    print("☁️  点云服务器已关闭")

def video_server(port=5000, hz=30):
    """
    视频发送服务器（发送测试图像）
    """
    print(f"🎥 启动视频服务器 - 端口: {port}, 频率: {hz}Hz")
    
    # 尝试导入OpenCV
    try:
        import cv2
        has_opencv = True
    except ImportError:
        print("⚠️  OpenCV未安装，将发送模拟数据")
        has_opencv = False
    
    # 创建TCP socket
    server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_sock.bind(('0.0.0.0', port))
    server_sock.listen(1)
    
    print(f"⏳ 等待客户端连接 (端口 {port})...")
    
    while True:
        try:
            client_sock, client_addr = server_sock.accept()
            print(f"✅ 视频客户端已连接: {client_addr}")
            
            frame_count = 0
            
            # 如果有OpenCV，尝试打开摄像头
            if has_opencv:
                cap = cv2.VideoCapture(0)
                if not cap.isOpened():
                    print("⚠️  无法打开摄像头，将生成测试图像")
                    cap = None
            else:
                cap = None
            
            while True:
                try:
                    if has_opencv:
                        if cap is not None:
                            # 从摄像头读取
                            ret, frame = cap.read()
                            if not ret:
                                # 生成测试图像
                                frame = generate_test_image(frame_count)
                        else:
                            # 生成测试图像
                            frame = generate_test_image(frame_count)
                        
                        # JPEG编码
                        _, encoded = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
                        data = encoded.tobytes()
                    else:
                        # 发送模拟数据
                        data = b'FAKE_VIDEO_FRAME_' + str(frame_count).encode() + b'\n'
                    
                    # 发送数据
                    client_sock.sendall(data)
                    
                    frame_count += 1
                    if frame_count % 30 == 0:
                        print(f"🎥 已发送视频帧: {frame_count} (大小: {len(data)/1024:.2f}KB)")
                    
                    time.sleep(1.0 / hz)
                    
                except (ConnectionResetError, BrokenPipeError):
                    print(f"⚠️  客户端断开连接")
                    break
                except Exception as e:
                    print(f"❌ 发送错误: {e}")
                    break
            
            if cap is not None:
                cap.release()
            
            client_sock.close()
            print(f"⏳ 等待新的客户端连接...")
            
        except KeyboardInterrupt:
            break
        except Exception as e:
            print(f"❌ 服务器错误: {e}")
            time.sleep(1)
    
    server_sock.close()
    print("🎥 视频服务器已关闭")

def generate_test_image(frame_count):
    """
    生成测试图像（如果没有OpenCV或摄像头）
    """
    try:
        import cv2
        import numpy as np
        
        # 创建640x480的彩色图像
        img = np.zeros((480, 640, 3), dtype=np.uint8)
        
        # 绘制渐变背景
        for i in range(480):
            img[i, :, 0] = int(255 * i / 480)  # 蓝色通道
            img[i, :, 1] = int(128)            # 绿色通道
            img[i, :, 2] = int(255 * (1 - i / 480))  # 红色通道
        
        # 绘制移动的圆形
        center_x = int(320 + 200 * np.sin(frame_count * 0.05))
        center_y = int(240 + 150 * np.cos(frame_count * 0.05))
        cv2.circle(img, (center_x, center_y), 50, (255, 255, 0), -1)
        
        # 绘制帧号
        cv2.putText(img, f'Frame: {frame_count}', (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        
        return img
    except:
        # 如果出错，返回空图像
        return np.zeros((480, 640, 3), dtype=np.uint8)

def main():
    parser = argparse.ArgumentParser(description='视频和点云测试服务器')
    parser.add_argument('--video-port', type=int, default=5000, help='视频端口 (默认: 5000)')
    parser.add_argument('--pointcloud-port', type=int, default=5001, help='点云端口 (默认: 5001)')
    parser.add_argument('--video-hz', type=int, default=30, help='视频帧率 (默认: 30)')
    parser.add_argument('--pointcloud-hz', type=int, default=10, help='点云帧率 (默认: 10)')
    parser.add_argument('--pcd-file', type=str, default=None, help='PCD文件路径（可选）')
    parser.add_argument('--no-video', action='store_true', help='禁用视频服务器')
    parser.add_argument('--no-pointcloud', action='store_true', help='禁用点云服务器')
    
    args = parser.parse_args()
    
    print("=" * 60)
    print("🚀 视频和点云测试服务器")
    print("=" * 60)
    print(f"配置:")
    if not args.no_video:
        print(f"  🎥 视频: 端口 {args.video_port}, {args.video_hz}Hz")
    if not args.no_pointcloud:
        print(f"  ☁️  点云: 端口 {args.pointcloud_port}, {args.pointcloud_hz}Hz")
    print("=" * 60)
    print("按 Ctrl+C 停止服务器")
    print("")
    
    threads = []
    
    # 启动视频服务器线程
    if not args.no_video:
        video_thread = threading.Thread(
            target=video_server,
            args=(args.video_port, args.video_hz),
            daemon=True
        )
        video_thread.start()
        threads.append(video_thread)
    
    # 启动点云服务器线程
    if not args.no_pointcloud:
        pointcloud_thread = threading.Thread(
            target=pointcloud_server,
            args=(args.pointcloud_port, args.pcd_file, args.pointcloud_hz),
            daemon=True
        )
        pointcloud_thread.start()
        threads.append(pointcloud_thread)
    
    if not threads:
        print("❌ 错误: 必须至少启用一个服务器（视频或点云）")
        return
    
    try:
        # 保持主线程运行
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n\n🛑 正在关闭服务器...")
    
    print("✅ 服务器已停止")

if __name__ == '__main__':
    main()

