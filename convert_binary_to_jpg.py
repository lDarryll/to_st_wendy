from PIL import Image
import numpy as np

# 读取 .bin 文件中的二进制数据
# bin_file_path = './key_point_out.bin'
# bin_file_path = './out_192_144.bin'
# bin_file_path = './out_384_288.bin'
# bin_file_path = './no_hand_1.bin'
bin_file_path = './input_net_img_onnx_no_sig_8_1.bin'
with open(bin_file_path, 'rb') as file:
    binary_data = file.read()

width = 192  # 替换为实际的图像宽度
height = 144  # 替换为实际的图像高度

# 如果图像是彩色图像 (RGB)，你需要按三通道来读取
image_array = np.frombuffer(binary_data, dtype=np.uint8).reshape((height, width, 3))

# 将 numpy 数组转换为图像对象
image = Image.fromarray(image_array)

# 保存图像为 .jpg 文件
jpg_file_path = './onnx_input_net_img_no_sig_8_1.jpg'
image.save(jpg_file_path)

print(f"Image saved as {jpg_file_path}")


