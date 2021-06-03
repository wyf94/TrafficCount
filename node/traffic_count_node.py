#!/home/wyf/anaconda3/envs/test/bin/python
#!coding=utf-8

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
import cv_bridge
from cv_bridge import CvBridge, CvBridgeError

import traffic_count.test.test as test
from traffic_count.detector.detector import Detector
import traffic_count.tracker.tracker as tracker


# import rospy
# from darknet_ros_msgs.msg import BoundingBoxes
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge
# import numpy as np
# import cv2


"""
#生成一个尺寸为size的图片mask，包含1个polygon，（值范围 0、1、2），供撞线计算使用
#list_point：点数组
#color_value: polygon填充的值
#size：图片尺寸
"""
def image_mask(list_point, color_value, size):
    # 根据视频尺寸，填充一个polygon，供撞线计算使用
    mask_image_temp = np.zeros(size, dtype=np.uint8)

    # 初始化撞线polygon
    ndarray_pts = np.array(list_point, np.int32)
    polygon_color_value = cv2.fillPoly(mask_image_temp, [ndarray_pts], color=color_value)
    polygon_color_value = polygon_color_value[:, :, np.newaxis]

    return polygon_color_value

def polygon_mask(point_list_first, point_list_second, size):
    polygon_value_first = image_mask(point_list_first, 1, size)
    polygon_value_second = image_mask(point_list_second, 2, size)
    
    # 撞线检测用mask，包含2个polygon，（值范围 0、1、2），供撞线计算使用
    polygon_mask_first_and_second = polygon_value_first + polygon_value_second

    # set the first  polygon to blue
    blue_color_plate = [255, 0, 0]
    blue_image = np.array(polygon_value_first * blue_color_plate, np.uint8)
    # set the first  polygon to yelllow
    yellow_color_plate = [0, 255, 255]
    yellow_image = np.array(polygon_value_second * yellow_color_plate, np.uint8)
    # 彩色图片（值范围 0-255）用于图片显示
    polygon_color_image = blue_image + yellow_image

    return polygon_mask_first_and_second,  polygon_color_image
    
def traffic_count(image, frame_count, list_bboxs, polygon_mask_first_and_second, first_list, second_list,  up_count, down_count):
    first_num = 0
    second_num = 0
    point_radius = 3

    if len(list_bboxs) > 0:
        for item_bbox in list_bboxs:
            x1, y1, x2, y2, cls_id, conf = item_bbox
            
            # 撞线的点(中心点)
            x = int(x1 + ((x2 - x1) * 0.5))
            y = int(y1 + ((y2 - y1) * 0.5))

            if polygon_mask_first_and_second[y, x] == 1 or polygon_mask_first_and_second[y, x]  ==3:
                first_num += 1
            elif polygon_mask_first_and_second[y, x] == 2:
                second_num += 1

            #画出中心list_bboxs的中心点
            list_pts = []
            list_pts.append([x-point_radius, y-point_radius])
            list_pts.append([x-point_radius, y+point_radius])
            list_pts.append([x+point_radius, y+point_radius])
            list_pts.append([x+point_radius, y-point_radius])
            ndarray_pts = np.array(list_pts, np.int32)
            image = cv2.fillPoly(image, [ndarray_pts], color=(0, 0, 255))           

        if frame_count > 2:
            second_list.pop(0)
            first_list.pop(0)

        first_list.append(first_num)
        second_list.append(second_num)

        if frame_count > 2 and first_list[0] > first_list[1]:
            first_diff = first_list[0] - first_list[1]
            second_diff =  second_list[1] - second_list[0]
            if first_diff == second_diff:
                up_count += first_diff
                print('up count:', up_count)
        elif frame_count >2 and second_list[0] > second_list[1]:
            second_diff =  second_list[0] - second_list[1]
            first_diff = first_list[1] - first_list[0]
            if first_diff == second_diff:
                down_count += first_diff  
                print('down count:', down_count)

    return up_count, down_count             

# def callback_image(data):

#     global detector
#     global up_count
#     global down_count
#     global blue_list
#     global yellow_list
#     global cur_frame
#     global polygon_mask_blue_and_yellow
#     global polygon_color_image

#     bridge = CvBridge()
#     cv_image = bridge.imgmsg_to_cv2(data,"bgr8")

#     bboxes = detector.detect(cv_image)

#     list_bboxs = []
#     for bbox in bboxes:
#             list_bboxs.append(bbox)

#     up_count, down_count = traffic_count(cv_image, cur_frame,  list_bboxs, polygon_mask_blue_and_yellow, blue_list, yellow_list,  up_count, down_count)

#     cv_image = cv2.add(cv_image, polygon_color_image)

#     font_draw_number = cv2.FONT_HERSHEY_SIMPLEX
#     # draw_text_postion = (int(960 * 0.01), int(540 * 0.05))
#     draw_text_postion = (int(1920 * 0.01), int(1080 * 0.05))
#     text_draw = 'DOWN: ' + str(down_count) + ' , UP: ' + str(up_count)
    
#     cv_image = cv2.putText(img=cv_image, text=text_draw,
#                                         org=draw_text_postion,
#                                         fontFace=font_draw_number,
#                                         fontScale=1, color=(0, 0, 255), thickness=2)
#     cur_frame += 1
#     #Display Image
#     cv2.imshow("traffic count", cv_image)
#     cv2.waitKey(1)

# # def callback_count(data):
# #     list_bboxs = []
# #     for box in data.bounding_boxes:
# # 	    list_bboxs.append(np.array([box.xmin, box.ymin, box.xmax, box.ymax, box.Calss,  round(box.probability, 2)]))
# # 	list_bboxs = np.array(list_bboxs)

# #     up_count, down_count = traffic_count(img_cv_rgb, cur_frame,  list_bboxs, polygon_mask_blue_and_yellow, blue_list, yellow_list,  up_count, down_count)

    
# def main():
#     while not rospy.is_shutdown():
#     	#Initialize ROS node
#         rospy.init_node('traffic_count', anonymous=False)
#     rate = rospy.Rate(10)
#     print("test1")
#     point_blue =  [[300*2, 380*2],[800*2, 380*2], [800*2, 390*2],[300*2, 390*2]]
#     point_yellow =   [[300*2, 370*2],[800*2, 370*2], [800*2, 380*2],[300*2, 380*2]]

#     polygon_mask_blue_and_yellow,  polygon_color_image = polygon_mask(point_blue, point_yellow,(1080, 1920))
    
#     # list 与蓝色polygon重叠
#     blue_list = []
#     # list 与黄色polygon重叠
#     yellow_list = []
#     # 进入数量
#     down_count = 0
#     # 离开数量
#     up_count = 0

#     cur_frame = 0 
#     print("test2")
#     # 初始化 yolov5
#     detector = Detector()
#     print("test3")
#     #Subscribe to image topic

#     image_sub = rospy.Subscriber('/image_source', Image, callback_image)
# 	# image_sub = rospy.Subscriber("camera_topic",Image, callback_image)
#     #Subscribe to darknet_ros to get BoundingBoxes from YOLOv3
# 	# sub_detection = rospy.Subscriber("detection_topic", BoundingBoxes , callback_count)

#     # pub_traffic_count = rospy.Publisher("count_topic", IntList, queue_size=10)
# 	# #print(msg) #Testing msg that is published
#     # pub_traffic_count.publish(msg)
#     rate.sleep()
#     rospy.spin()

# if __name__ == '__main__':
#     try :
#         rospy.loginfo("Starting traffic count node")
#         main()
#     except rospy.ROSInterruptException:
#         rospy.loginfo( "Shutting down traffic count  node.")
#         cv2.destroyAllWindows()
#         pass




def callback(data):
    global detector

    global up_count
    global down_count
    global blue_list
    global yellow_list
    global cur_frame
    global polygon_mask_blue_and_yellow
    global polygon_color_image

    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(data,"bgr8")

    bboxes = detector.detect(cv_image)

    list_bboxs = []
    for bbox in bboxes:
        list_bboxs.append(bbox)

    up_count, down_count = traffic_count(cv_image, cur_frame,  list_bboxs, polygon_mask_blue_and_yellow, blue_list, yellow_list,  up_count, down_count)
    
    font_draw_number = cv2.FONT_HERSHEY_SIMPLEX
    # draw_text_postion = (int(960 * 0.01), int(540 * 0.05))
    draw_text_postion = (int(1920 * 0.01), int(1080 * 0.05))
    text_draw = 'DOWN: ' + str(down_count) + ' , UP: ' + str(up_count)

    cv_image = cv2.add(cv_image, polygon_color_image)
    cv_image = cv2.putText(img=cv_image, text=text_draw,
                                        org=draw_text_postion,
                                        fontFace=font_draw_number,
                                        fontScale=1, color=(0, 0, 255), thickness=2)    

    # print("bboxes:", text_draw)
    cv2.imshow("lala",cv_image)
    cur_frame += 1
    cv2.waitKey(1)

def main():
    rospy.init_node('showImage',anonymous = True)
    rospy.Subscriber('/image_source', Image, callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        rospy.loginfo("Starting cv_bridge_test node")
        count = 0
        # 初始化 yolov5
        detector = Detector()

        point_blue =  [[300*2, 380*2],[800*2, 380*2], [800*2, 390*2],[300*2, 390*2]]
        point_yellow =   [[300*2, 370*2],[800*2, 370*2], [800*2, 380*2],[300*2, 380*2]]

        polygon_mask_blue_and_yellow,  polygon_color_image = polygon_mask(point_blue, point_yellow,(1080, 1920))

        # list 与蓝色polygon重叠
        blue_list = []
        # list 与黄色polygon重叠
        yellow_list = []
        # 进入数量
        down_count = 0
        # 离开数量
        up_count = 0

        cur_frame = 0 
        main() 
    except KeyboardInterrupt:
        print( "Shutting down cv_bridge_test node.")
        cv2.destroyAllWindows()

