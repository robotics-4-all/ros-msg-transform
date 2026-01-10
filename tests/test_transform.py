import unittest
from std_msgs.msg import String, Int32, Float64, Bool, Header, Int32MultiArray
from geometry_msgs.msg import Pose, Point, Quaternion
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse
from ros_msg_transform import (
    dict_to_ros_msg, ros_msg_to_dict,
    dict_to_ros_srv_request, dict_to_ros_srv_response,
    ros_srv_req_to_dict, ros_srv_resp_to_dict,
    fill_ros_message
)
import base64

class TestRosMsgTransform(unittest.TestCase):

    def test_string_msg(self):
        msg_type = 'std_msgs/String'
        d = {'data': 'hello world'}
        msg = dict_to_ros_msg(msg_type, d)
        self.assertIsInstance(msg, String)
        self.assertEqual(msg.data, 'hello world')
        
        d_back = ros_msg_to_dict(msg)
        self.assertEqual(d_back, d)

    def test_int32_msg(self):
        msg_type = 'std_msgs/Int32'
        d = {'data': 42}
        msg = dict_to_ros_msg(msg_type, d)
        self.assertIsInstance(msg, Int32)
        self.assertEqual(msg.data, 42)
        
        d_back = ros_msg_to_dict(msg)
        self.assertEqual(d_back, d)

    def test_float64_msg(self):
        msg_type = 'std_msgs/Float64'
        d = {'data': 3.14}
        msg = dict_to_ros_msg(msg_type, d)
        self.assertIsInstance(msg, Float64)
        self.assertEqual(msg.data, 3.14)
        
        d_back = ros_msg_to_dict(msg)
        self.assertEqual(d_back, d)

    def test_bool_msg(self):
        msg_type = 'std_msgs/Bool'
        d = {'data': True}
        msg = dict_to_ros_msg(msg_type, d)
        self.assertIsInstance(msg, Bool)
        self.assertTrue(msg.data)
        
        d_back = ros_msg_to_dict(msg)
        self.assertEqual(d_back, d)

    def test_nested_msg(self):
        msg_type = 'geometry_msgs/Pose'
        d = {
            'position': {'x': 1.0, 'y': 2.0, 'z': 3.0},
            'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}
        }
        msg = dict_to_ros_msg(msg_type, d)
        self.assertIsInstance(msg, Pose)
        self.assertEqual(msg.position.x, 1.0)
        self.assertEqual(msg.orientation.w, 1.0)
        
        d_back = ros_msg_to_dict(msg)
        self.assertEqual(d_back, d)

    def test_array_msg(self):
        msg_type = 'std_msgs/Int32MultiArray'
        d = {'data': [1, 2, 3, 4]}
        msg = dict_to_ros_msg(msg_type, d)
        self.assertIsInstance(msg, Int32MultiArray)
        self.assertEqual(list(msg.data), [1, 2, 3, 4])
        
        d_back = ros_msg_to_dict(msg)
        self.assertEqual(d_back['data'], [1, 2, 3, 4])

    def test_time_msg(self):
        # Header contains time
        msg_type = 'std_msgs/Header'
        d = {
            'seq': 1,
            'stamp': {'secs': 100, 'nsecs': 200},
            'frame_id': 'map'
        }
        msg = dict_to_ros_msg(msg_type, d)
        self.assertIsInstance(msg, Header)
        self.assertEqual(msg.stamp.secs, 100)
        self.assertEqual(msg.stamp.nsecs, 200)
        
        d_back = ros_msg_to_dict(msg)
        self.assertEqual(d_back['stamp']['secs'], 100)
        self.assertEqual(d_back['stamp']['nsecs'], 200)

    def test_binary_msg(self):
        msg_type = 'sensor_msgs/Image'
        # uint8[] data
        raw_data = bytes([1, 2, 3, 4])
        encoded_data = base64.standard_b64encode(raw_data).decode('utf-8')
        d = {
            'header': {'seq': 0, 'stamp': {'secs': 0, 'nsecs': 0}, 'frame_id': ''},
            'height': 1,
            'width': 4,
            'encoding': 'rgb8',
            'is_bigendian': 0,
            'step': 4,
            'data': encoded_data
        }
        msg = dict_to_ros_msg(msg_type, d)
        self.assertIsInstance(msg, Image)
        self.assertEqual(list(bytearray(msg.data)), [1, 2, 3, 4])
        
        d_back = ros_msg_to_dict(msg)
        self.assertEqual(d_back['data'], encoded_data)

    def test_service_request(self):
        srv_type = 'std_srvs/Trigger'
        d = {}
        req = dict_to_ros_srv_request(srv_type, d)
        self.assertIsInstance(req, TriggerRequest)
        
        d_back = ros_srv_req_to_dict(req)
        self.assertEqual(d_back, d)

    def test_service_response(self):
        srv_type = 'std_srvs/Trigger'
        d = {'success': True, 'message': 'done'}
        resp = dict_to_ros_srv_response(srv_type, d)
        self.assertIsInstance(resp, TriggerResponse)
        self.assertTrue(resp.success)
        self.assertEqual(resp.message, 'done')
        
        d_back = ros_srv_resp_to_dict(resp)
        self.assertEqual(d_back, d)

    def test_fill_ros_message(self):
        msg = String()
        d = {'data': 'filled'}
        fill_ros_message(msg, d)
        self.assertEqual(msg.data, 'filled')

    def test_invalid_field(self):
        msg_type = 'std_msgs/String'
        d = {'invalid_field': 'value'}
        with self.assertRaises(ValueError) as cm:
            dict_to_ros_msg(msg_type, d)
        self.assertIn('ROS message has no field named: invalid_field', str(cm.exception))

if __name__ == '__main__':
    unittest.main()
