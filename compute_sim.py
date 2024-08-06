import numpy as np

def load_bin_file(file_path, dtype, shape):
    return np.fromfile(file_path, dtype=dtype).reshape(shape)

def cosine_similarity(vec1, vec2):
    dot_product = np.dot(vec1, vec2)
    norm_vec1   = np.linalg.norm(vec1)
    norm_vec2   = np.linalg.norm(vec2)
    return dot_product / (norm_vec1 * norm_vec2)

# 加载两个bin文件
float32_file_path = './boxes.bin'
int8_file_path    = './boxes_st.bin'
# float32_file_path = './confidences.bin'
# int8_file_path    = './confidences_st.bin'

# 假设文件大小为384x2
shape = (384, 4)

float32_data = load_bin_file(float32_file_path, dtype=np.float32, shape=shape)
int8_data    = load_bin_file(int8_file_path, dtype=np.int8, shape=shape).astype(np.float32)

similarity   = cosine_similarity(float32_data.flatten(), int8_data.flatten())
print(f"Cosine similarity: {similarity}")
