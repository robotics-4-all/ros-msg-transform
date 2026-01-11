# ROS Message Transform Library

[![PyPI - Version](https://img.shields.io/pypi/v/ros-msg-transform.svg)](https://pypi.org/project/ros-msg-transform)
[![PyPI - Python Version](https://img.shields.io/pypi/pyversions/ros-msg-transform.svg)](https://pypi.org/project/ros-msg-transform)
[![License](https://img.shields.io/badge/License-MIT-blue.svg)](https://github.com/robotics4all/ros-msg-transform/blob/main/LICENSE)
[![GitHub Workflow Status](https://img.shields.io/github/actions/workflow/status/robotics4all/ros-msg-transform/main.yml?branch=main)](https://github.com/robotics4all/ros-msg-transform/actions?query=workflow%3A"Python+package")

## Overview

`ros-msg-transform` is a lightweight Python library designed to simplify the conversion between ROS messages and standard Python dictionaries. This library is particularly useful for developers working with ROS (Robot Operating System) who need to interact with ROS message data in a more flexible and Python-native way, or for integrating ROS systems with non-ROS components that expect dictionary-like data structures.

The library handles various ROS message types, including primitive types, nested messages, arrays, and even binary data (e.g., `sensor_msgs/Image`). It also provides utilities for converting ROS service requests and responses.

## Features

*   **Bidirectional Conversion**: Seamlessly convert ROS messages to Python dictionaries and vice-versa.
*   **Support for Standard ROS Types**: Handles common ROS message primitives (`String`, `Int32`, `Float64`, `Bool`, etc.).
*   **Nested Message Handling**: Correctly processes messages with nested structures (e.g., `geometry_msgs/Pose`).
*   **Array Support**: Converts ROS array fields to Python lists (e.g., `std_msgs/Int32MultiArray`).
*   **Time and Header Support**: Properly transforms ROS `Header` and `Time` types.
*   **Binary Data Encoding/Decoding**: Automatically handles `uint8[]` fields (e.g., `sensor_msgs/Image`) by encoding them to/from Base64 strings.
*   **ROS Service Conversion**: Functions for converting ROS service requests and responses to/from dictionaries.
*   **In-place Message Filling**: Utility to fill an existing ROS message object with data from a dictionary.
*   **Error Handling**: Provides informative errors for invalid message fields.

## Installation

You can install `ros-msg-transform` using pip:

```bash
pip install ros-msg-transform
```

For development purposes, you can install the package with its development dependencies:

```bash
pip install "ros-msg-transform[dev]"
```

## Usage

### Converting ROS Message to Dictionary

```python
from std_msgs.msg import String
from ros_msg_transform import ros_msg_to_dict

ros_string_msg = String(data="Hello ROS!")
python_dict = ros_msg_to_dict(ros_string_msg)

print(python_dict)
# Expected output: {'data': 'Hello ROS!'}
```

### Converting Dictionary to ROS Message

```python
from geometry_msgs.msg import Pose
from ros_msg_transform import dict_to_ros_msg

pose_dict = {
    "position": {"x": 1.0, "y": 2.0, "z": 3.0},
    "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
}
ros_pose_msg = dict_to_ros_msg("geometry_msgs/Pose", pose_dict)

print(ros_pose_msg)
# Expected output: position: 
#   x: 1.0
#   y: 2.0
#   z: 3.0
# orientation: 
#   x: 0.0
#   y: 0.0
#   z: 0.0
#   w: 1.0
```

### Handling Binary Data (e.g., `sensor_msgs/Image`)

When converting a `sensor_msgs/Image` to a dictionary, the `data` field (which is `uint8[]`) is automatically Base64 encoded. When converting a dictionary back to an `Image` message, a Base64 encoded string in the `data` field is automatically decoded.

```python
import base64
from sensor_msgs.msg import Image
from ros_msg_transform import dict_to_ros_msg, ros_msg_to_dict

# Example: Dictionary to Image message
raw_image_data = bytes([0xFF, 0xD8, 0xFF, 0xE0, 0x00, 0x10, 0x4A, 0x46]) # JPEG header example
encoded_image_data = base64.b64encode(raw_image_data).decode('utf-8')

image_dict = {
    "header": {"seq": 0, "stamp": {"secs": 0, "nsecs": 0}, "frame_id": "camera"},
    "height": 1,
    "width": 8,
    "encoding": "jpeg",
    "is_bigendian": 0,
    "step": 8,
    "data": encoded_image_data,
}

ros_image_msg = dict_to_ros_msg("sensor_msgs/Image", image_dict)
print(f"Decoded image data length: {len(ros_image_msg.data)}")
# Expected output: Decoded image data length: 8

# Example: Image message to Dictionary
dict_from_image_msg = ros_msg_to_dict(ros_image_msg)
print(f"Encoded image data: {dict_from_image_msg['data']}")
# Expected output: Encoded image data: /fjoAAAEoRg==
```

### Filling an existing ROS Message

```python
from std_msgs.msg import String
from ros_msg_transform import fill_ros_message

existing_msg = String(data="Initial value")
new_data = {"data": "Updated value"}
fill_ros_message(existing_msg, new_data)

print(existing_msg.data)
# Expected output: Updated value
```

### Service Request/Response Conversion

```python
from std_srvs.srv import TriggerRequest, TriggerResponse
from ros_msg_transform import (
    dict_to_ros_srv_request,
    ros_srv_req_to_dict,
    dict_to_ros_srv_response,
    ros_srv_resp_to_dict,
)

# Request
trigger_req_dict = {}
ros_trigger_req = dict_to_ros_srv_request("std_srvs/Trigger", trigger_req_dict)
print(ros_trigger_req)
# Expected output: <std_srvs.srv._Trigger.TriggerRequest object at ...>

dict_from_ros_req = ros_srv_req_to_dict(ros_trigger_req)
print(dict_from_ros_req)
# Expected output: {}

# Response
trigger_resp_dict = {"success": True, "message": "Triggered successfully"}
ros_trigger_resp = dict_to_ros_srv_response("std_srvs/Trigger", trigger_resp_dict)
print(ros_trigger_resp.success, ros_trigger_resp.message)
# Expected output: True Triggered successfully

dict_from_ros_resp = ros_srv_resp_to_dict(ros_trigger_resp)
print(dict_from_ros_resp)
# Expected output: {'success': True, 'message': 'Triggered successfully'}
```

## Development

### Setup

1.  Clone the repository:
    ```bash
    git clone https://github.com/robotics4all/ros-msg-transform.git
    cd ros-msg-transform
    ```
2.  Create and activate a virtual environment:
    ```bash
    python3 -m venv .venv
    source .venv/bin/activate
    ```
3.  Install the package with development dependencies:
    ```bash
    pip install ".[dev]"
    ```

### Running Tests

To run the test suite:

```bash
make test
```

### Version Bumping

This project uses `bumpversion` for managing versions. To bump the version:

*   **Major version**: `make bump-major`
*   **Minor version**: `make bump-minor`
*   **Patch version**: `make bump-patch`

These commands will update the version number in `pyproject.toml` and `ros_msg_transform/__init__.py`, create a git commit, and add a git tag.

## Contributing

We welcome contributions! Please refer to our [Contributing Guidelines](CONTRIBUTING.md) (if available) for more information on how to get started.

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Contact

For any questions or suggestions, please open an issue on the [GitHub repository](https://github.com/robotics4all/ros-msg-transform).