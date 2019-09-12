import math 

'''
Input: A box center at position (x,y) and its width, height in their respective directions
ei box = [x, y, width, height]
'''

def IOU(box1, box2):
    s1_upper_x = box1[0] + (box1[2] / 2.0);
    s1_upper_y = box1[1] + (box1[3] / 2.0);
    s1_lower_x = box1[0] - (box1[2] / 2.0);
    s1_lower_y = box1[1] - (box1[3] / 2.0);

    s2_upper_x = box2[0] + (box2[2] / 2.0);
    s2_upper_y = box2[1] + (box2[3] / 2.0);
    s2_lower_x = box2[0] - (box2[2] / 2.0);
    s2_lower_y = box2[1] - (box2[3] / 2.0);

    x_u = min(s1_upper_x, s2_upper_x);
    y_u = min(s1_upper_y, s2_upper_y);

    x_l = max(s1_lower_x, s2_lower_x);
    y_l = max(s1_lower_y, s2_lower_y);

    intersect = (y_u - y_l) * (x_u - x_l);
    return intersect


if __name__ == '__main__':
    t1 = IOU([3.5, 4.0, 3.0, 4.0], [3.5, 5.5, 3.0, 3.0]) == 6.0
    t2 = IOU([4.5, 3.0, 5.0, 2.0], [4.5, 4.5, 3, 3]) == 3.0
    t3 = IOU([4.5, 3.5, 3.0, 3.0], [2.5, 5.5, 3, 3]) == 1.0

    if t1 and t2 and t3:
        print ("Test was successful")
    else:
        print ("FAILED to find intersect between two bounding boxes")
