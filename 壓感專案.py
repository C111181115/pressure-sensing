import serial
import numpy as np
from PIL import Image
from collections import deque

# 計算CRC，這是一個簡單的CRC算法，根據需求可以修改
def calculate_crc(data):
    crc = 0X01 #0XAA+0XAB+0XAC=0X201
    for byte in data:
        crc |= byte
    return crc

# UART參數設置
uart_port = 'COM3'  # 替換為你的UART端口
baud_rate = 115200
timeout = 1

# 連接UART
ser = serial.Serial(uart_port, baudrate=baud_rate, timeout=timeout)

# 函數逐個byte讀取直到檢測到header
def find_header():
    header = [0xAA, 0xAB, 0xAC]

    buffer = deque(maxlen=3)  # 使用deque作為隊列，最大長度為3

    while True:
        byte = ser.read(1)  # 逐個讀取字節
        print(byte, sep=" ")
        if byte:
            buffer.append(ord(byte))  # 將讀取到的字節添加到隊列

        # 檢查隊列是否等於標頭
        if list(buffer) == header:
            print("找到標頭:", list(buffer))
            return  # 找到header，返回

# 讀取16x16二維陣列並檢查CRC
def read_uart_frame():
    # 先找header
    find_header()

    # 讀取16x16陣列，每個元素2個位元組
    data = ser.read(16 * 16 * 2)

    # 讀取CRC
    received_crc = ser.read(1)
    print(received_crc, sep=" ")
    # 計算接收到的數據的CRC
    crc_code = calculate_crc(data)
    print("CRC:", received_crc[0])
    print("calculated_crc:", crc_code)

    # 驗證CRC
    if crc_code == received_crc[0]:
        print("CRC matched, processing frame.")
        
        # CRC正確，處理數據
        array = np.zeros((16, 16), dtype=np.uint16)
        for i in range(16):
            for j in range(16):
                high_byte = data[(i * 16 + j) * 2]
                low_byte = data[(i * 16 + j) * 2 + 1]
                array[i, j] = (high_byte << 8) + low_byte  # 組合高位和低位成16位元整數

        # 將數據轉換為影像
        image_array = (array / 65535.0 * 255).astype(np.uint8)
        img = Image.fromarray(image_array, mode='L')  # 'L'表示灰度影像
        img = img.resize((256, 256), Image.NEAREST)  # 放大影像方便顯示
        img.show()
    else:
        print("CRC mismatch, frame ignored.")

# 持續讀取UART並處理連續的二維陣列
image_count = 0  # 計數器初始化

try:
    while image_count < 10:  # 限制圖片生成次數為10次
        read_uart_frame()
        image_count += 1  # 每生成一張圖片，計數器加1
except KeyboardInterrupt:
    # 關閉UART
    ser.close()
    print("UART connection closed.")
