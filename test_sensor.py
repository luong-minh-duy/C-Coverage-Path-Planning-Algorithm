import pytest
from shapely.geometry import Polygon, Point
from sensor import sense_environment

def test_sense_inside_free_area():
    # Môi trường tự do là hình vuông lớn
    env_free = Polygon([(0,0), (0,10), (10,10), (10,0)])
    current_pos = (2, 2)
    next_pos = (8, 2)
    range_sensor = 1.0

    result = sense_environment(current_pos, next_pos, range_sensor, env_free)
    
    assert result.area > 0
    assert result.within(env_free)

def test_sense_outside_free_area():
    # Môi trường tự do nhỏ hơn vùng quét
    env_free = Polygon([(4,4), (4,5), (5,5), (5,4)])  # vùng nhỏ nằm trong buffer
    current_pos = (2,2)
    next_pos = (6,2)
    range_sensor = 2.0

    result = sense_environment(current_pos, next_pos, range_sensor, env_free)
    print("result: " + str(result.area))
    
    # Kết quả phải là phần giao giữa buffer và env_free
    assert result.area > 0
    assert result.intersects(env_free)
    assert result.within(env_free.buffer(0.1))  # cho phép sai số nhỏ

def test_no_intersection():
    # Vùng tự do không giao vùng quét
    env_free = Polygon([(10,10), (10,11), (11,11), (11,10)])  # rất xa
    current_pos = (0, 0)
    next_pos = (1, 0)
    range_sensor = 1.0

    result = sense_environment(current_pos, next_pos, range_sensor, env_free)

    assert result.is_empty

