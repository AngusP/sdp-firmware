from collections import OrderedDict


def get_rotation_delta(angle):
    if angle < 0:
        return get_rotation_delta(-angle)

    data = OrderedDict([
        (0, 0),
        (45, 48),
        (90, 128),
        (180, 288),
        (360, 628),
        (720, 1310)
    ])

    if angle in data:
        return data[angle]

    smaller_angle = 0
    larger_angle = 0
    i = 0

    for measured_angle in data:
        i += 1
        if measured_angle > angle or i == len(data):
            larger_angle = measured_angle
            break
        smaller_angle = measured_angle

    gradient = (data[larger_angle] - data[smaller_angle]) / (larger_angle - smaller_angle)

    return int(gradient * angle - gradient * smaller_angle + data[smaller_angle])

