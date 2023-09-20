import math
import numpy


def cal_avg(numbers):
    chuck_num = []
    for i in range(0, 60, 10):
        chuck_num.append(numbers[i : i + 10])
    for i in range(310, 360, 10):
        chuck_num.append(numbers[i : i + 10])
    for i in range(60, 310, 10):
        chuck_num.append(numbers[i : i + 10])

    averages = []
    for chuck in chuck_num:
        valid_num = [num for num in chuck if num != 0.0]
        if valid_num:
            avg = sum(valid_num) / len(valid_num)
            averages.append(avg)
        else:
            averages.append(2.0)
    return averages


def get_key():
    # Print terminal message and get inputs
    input_theta = float(input("Input : "))

    if input_theta > 180:
        input_theta = input_theta % 180
    elif input_theta < -180:
        input_theta = input_theta % -180
    print(input_theta)
    input_theta = numpy.deg2rad(input_theta)
    print(input_theta)

    return input_theta


print(90 % 180)
