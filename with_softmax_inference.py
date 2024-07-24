import numpy as np
import cv2
import onnxruntime
import math

class SSD_Detector:
    def __init__(self, model_path, input_height, input_width):
        self.model_path      = model_path
        self.input_height    = input_height
        self.input_width     = input_width
        self.sess            = onnxruntime.InferenceSession(model_path)

        self.priors          = []
        self.center_variance = 0.1  # 假设一个中心方差值
        self.size_variance   = 0.2  # 假设一个尺寸方差值
        self.generate_anchors(self.input_width, self.input_height)  # 只生成一次锚点
    
    def clip(self, value, max_value):
        return max(0.0, min(value, max_value))

    def generate_anchors(self, in_width, in_height):
        min_boxes       = [[24, 32, 48], [64, 96]]
        featuremap_size = [
            [12, 9],  # layer 11
            [6, 5]    # layer 13
        ]

        self.priors.clear()
        for index in range(len(featuremap_size)):
            for j in range(int(featuremap_size[index][1])):
                y_center = (j + 0.5) / featuremap_size[index][1]
                for i in range(int(featuremap_size[index][0])):
                    x_center = (i + 0.5) / featuremap_size[index][0]

                    for k in min_boxes[index]:
                        w = k / in_width
                        h = k / in_height
                        self.priors.append([
                            self.clip(x_center, 1.0),
                            self.clip(y_center, 1.0),
                            self.clip(w, 1.0),
                            self.clip(h, 1.0)
                        ])

        # print(f"HandDetector GenerateAnchors finished, num of anchors: {len(self.priors)}!")
        return len(self.priors)

    def generate_bbox(self, bbox_collection, scores, boxes, score_threshold, num_anchors):
        max_score = score_threshold
        best_bbox = None
        for i in range(num_anchors):
            if scores[i][1] > max_score:
                max_score = scores[i][1]
                x_center = boxes[i][0] * self.priors[i][2] * self.center_variance + self.priors[i][0]
                y_center = boxes[i][1] * self.priors[i][3] * self.center_variance + self.priors[i][1]
                w = math.exp(boxes[i][2] * self.size_variance) * self.priors[i][2]
                h = math.exp(boxes[i][3] * self.size_variance) * self.priors[i][3]

                x1 = self.clip(x_center - w / 2.0, 1.0)
                y1 = self.clip(y_center - h / 2.0, 1.0)
                x2 = self.clip(x_center + w / 2.0, 1.0)
                y2 = self.clip(y_center + h / 2.0, 1.0)
                best_bbox = [x1, y1, x2, y2, max_score]

        # if best_bbox is not None:
        #     bbox_collection.append(best_bbox)
        #     print(f"Generated bbox: x1={best_bbox[0]}, y1={best_bbox[1]}, x2={best_bbox[2]}, y2={best_bbox[3]}, score={best_bbox[4]}")

    def resize_image(self, src_img):
        src_h, src_w, c = src_img.shape
        if src_h / src_w > self.input_height / self.input_width:
            height_new  = self.input_height
            width_new   = int(self.input_height * src_w / src_h)
        else:
            height_new  = int(self.input_width * src_h / src_w)
            width_new   = self.input_width
        resized_img     = cv2.resize(src_img, (width_new, height_new))
        input_net_img   = np.zeros((self.input_height, self.input_width, 3), dtype=np.uint8)
        input_net_img[:height_new, :width_new, ::-1] = resized_img  # bgr - > rgb
        

        return input_net_img
    
    def preprocess(self, src_img):
        input_net_img = self.resize_image(src_img)
        cv2.imwrite("./pre.jpg", input_net_img)
        input_net_img = input_net_img.transpose(2, 0, 1)  # 将图像的轴顺序从 (height, width, channels) 转换为 (channels, height, width)

        flattened_rgb_data = input_net_img.copy().flatten()
        output_path        = './input_net_img.bin'
        with open(output_path, 'wb') as binary_file:
            binary_file.write(flattened_rgb_data.tobytes())
        print("Preprocessed image has been converted to binary file successfully.")

        input_net_img = input_net_img.astype(np.float32)
        input_net_img = np.expand_dims(input_net_img, axis=0)
        
        return input_net_img

    def inference(self, src_img):
        input_net_img      = self.preprocess(src_img)
        input_name         = self.sess.get_inputs()[0].name
        confidences, boxes = self.sess.run(None, {input_name: input_net_img})
        print(confidences)
        # print(f"confidences shape: {confidences.shape}")
        # print(f"boxes shape: {boxes.shape}")
        return confidences[0], boxes[0]

    def postprocess(self, confidences, boxes, conf):
        bboxes_collection = []
        self.generate_bbox(bboxes_collection, confidences, boxes, conf, len(self.priors))
        return bboxes_collection
    
    def draw_result(self, img, bboxes):
        # Define colors
        green_color  = (0, 255, 0)  # BGR format
        red_color    = (0, 0, 255)
        blue_color   = (255, 0, 0)
        
        point_radius = 3
        thickness    = -1  # Fill the circle

        h, w = img.shape[:2]
        for bbox in bboxes:
            x1 = bbox[0] * w
            y1 = bbox[1] * h
            x2 = bbox[2] * w
            y2 = bbox[3] * h
            
            print(f"x1:{x1}, y1:{y1}, x2:{x2}, y2:{y2}, conf:{bbox[4]}")
            cv2.rectangle(img, (int(x1), int(y1)), (int(x2), int(y2)), red_color, 2)
            
        cv2.imwrite("./result_one_hand_quant_per_channel.jpg", img)

if __name__ == "__main__":

    input_height       = 144
    input_width        = 192
    box_conf           = 0.9
    input_img_path     = "../imgs/21_1692932849057.jpg"
    onnx_path          = "/mnt/sdc/DataSSD/Depth/zs_space/works_space/tutorials/notebooks/models/2_fox_complex_144_light_RGB_noprepro_QDQ_quant.onnx"

    detector           = SSD_Detector(onnx_path, input_height, input_width)
    src_img            = cv2.imread(input_img_path)
    confidences, boxes = detector.inference(src_img)
    bboxes_result      = detector.postprocess(confidences, boxes, box_conf)

    # print(len(bboxes_result))
    # print(bboxes_result)
    detector.draw_result(src_img, bboxes_result)
      

