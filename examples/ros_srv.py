#!/usr/bin/env python3

from ros_msg_transform import (
    dict_to_ros_srv_request,
    dict_to_ros_srv_response,
    ros_srv_req_to_dict,
    ros_srv_resp_to_dict
)


if __name__ == '__main__':
    srv_req = dict_to_ros_srv_request('std_srvs/Trigger', {})
    print('Dictionary to ROS Service [std_srvs/Trigger] request: %s' % srv_req)
    srv_resp = dict_to_ros_srv_response('std_srvs/Trigger',
                                        {'success': True, 'message': 'Do it!'})
    print('Dictionary to ROS Service [std_srvs/Trigger] response: %s'
          % srv_resp)
    d = ros_srv_req_to_dict(srv_req)
    print('ROS Service [std_srvs/Trigger] request to dict: %s' % d)
    d = ros_srv_resp_to_dict(srv_resp)
    print('ROS Service [std_srvs/Trigger] response to dict: %s' % d)
