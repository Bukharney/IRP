import math
import random
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


def fuzzy_rule(sensor_membership):
    rule = {}
    rule[0] = sensor_membership[0]
    rule[1] = sensor_membership[2] * -0.5
    rule[2] = sensor_membership[3] * 0.5
    rule[3] = sensor_membership[4] * -0.25
    rule[4] = sensor_membership[5] * 0.25

    angular = rule[1] + rule[2] + rule[3] + rule[4]
    linear = rule[0]

    return linear, angular


# test fuzzy_rule
def test_fuzzy_rule():
    random.seed(0)
    for i in range(7):
        sensor = [random.random() for i in range(7)]
        linear, angular = fuzzy_rule(sensor)
        print("linear : ", linear)
        print("angular : ", angular)


test_fuzzy_rule()
