// Auto-generated. Do not edit!

// (in-package a1_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let FootPlanDiscrete = require('./FootPlanDiscrete.js');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class MultiFootPlanDiscrete {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.feet = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('feet')) {
        this.feet = initObj.feet
      }
      else {
        this.feet = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type MultiFootPlanDiscrete
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [feet]
    // Serialize the length for message field [feet]
    bufferOffset = _serializer.uint32(obj.feet.length, buffer, bufferOffset);
    obj.feet.forEach((val) => {
      bufferOffset = FootPlanDiscrete.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type MultiFootPlanDiscrete
    let len;
    let data = new MultiFootPlanDiscrete(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [feet]
    // Deserialize array length for message field [feet]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.feet = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.feet[i] = FootPlanDiscrete.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    object.feet.forEach((val) => {
      length += FootPlanDiscrete.getMessageSize(val);
    });
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'a1_msgs/MultiFootPlanDiscrete';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '24cfed7044607c942541950144570913';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # This is a message to hold a discrete foot plan for multiple robot feet
    #
    # The plans of each foot are stored in a vector of FootPlanDiscrete messages
    # (0 = front left, 1 = back left, 2 = front right, 3 = back right).
    # Accurate timing information to localize the plans is stored in the header
    
    Header header
    a1_msgs/FootPlanDiscrete[] feet
    
    ================================================================================
    MSG: std_msgs/Header
    # Standard metadata for higher-level stamped data types.
    # This is generally used to communicate timestamped data 
    # in a particular coordinate frame.
    # 
    # sequence ID: consecutively increasing ID 
    uint32 seq
    #Two-integer timestamp that is expressed as:
    # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    # time-handling sugar is provided by the client library
    time stamp
    #Frame this data is associated with
    string frame_id
    
    ================================================================================
    MSG: a1_msgs/FootPlanDiscrete
    # This is a message to hold the discrete foot plan for a single robot foot
    #
    # The plan is defined as a vector of FootState messages
    # Accurate timing information is stored in the header
    
    Header header
    a1_msgs/FootState[] footholds
    
    ================================================================================
    MSG: a1_msgs/FootState
    # This is a message to hold the state of a single foot of a legged robot
    #
    # The states of each foot are stored in a vector of FootState messages
    # Accurate timing information is stored in the header
    
    Header header
    uint32 traj_index
    geometry_msgs/Vector3 position
    geometry_msgs/Vector3 velocity
    geometry_msgs/Vector3 acceleration
    bool contact
    
    ================================================================================
    MSG: geometry_msgs/Vector3
    # This represents a vector in free space. 
    # It is only meant to represent a direction. Therefore, it does not
    # make sense to apply a translation to it (e.g., when applying a 
    # generic rigid transformation to a Vector3, tf2 will only apply the
    # rotation). If you want your data to be translatable too, use the
    # geometry_msgs/Point message instead.
    
    float64 x
    float64 y
    float64 z
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new MultiFootPlanDiscrete(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.feet !== undefined) {
      resolved.feet = new Array(msg.feet.length);
      for (let i = 0; i < resolved.feet.length; ++i) {
        resolved.feet[i] = FootPlanDiscrete.Resolve(msg.feet[i]);
      }
    }
    else {
      resolved.feet = []
    }

    return resolved;
    }
};

module.exports = MultiFootPlanDiscrete;
