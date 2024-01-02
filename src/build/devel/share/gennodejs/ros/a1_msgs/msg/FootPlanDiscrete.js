// Auto-generated. Do not edit!

// (in-package a1_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let FootState = require('./FootState.js');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class FootPlanDiscrete {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.footholds = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('footholds')) {
        this.footholds = initObj.footholds
      }
      else {
        this.footholds = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type FootPlanDiscrete
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [footholds]
    // Serialize the length for message field [footholds]
    bufferOffset = _serializer.uint32(obj.footholds.length, buffer, bufferOffset);
    obj.footholds.forEach((val) => {
      bufferOffset = FootState.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type FootPlanDiscrete
    let len;
    let data = new FootPlanDiscrete(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [footholds]
    // Deserialize array length for message field [footholds]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.footholds = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.footholds[i] = FootState.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    object.footholds.forEach((val) => {
      length += FootState.getMessageSize(val);
    });
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'a1_msgs/FootPlanDiscrete';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '2ae5c98d4a7907bd7964a0959d454c94';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # This is a message to hold the discrete foot plan for a single robot foot
    #
    # The plan is defined as a vector of FootState messages
    # Accurate timing information is stored in the header
    
    Header header
    a1_msgs/FootState[] footholds
    
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
    const resolved = new FootPlanDiscrete(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.footholds !== undefined) {
      resolved.footholds = new Array(msg.footholds.length);
      for (let i = 0; i < resolved.footholds.length; ++i) {
        resolved.footholds[i] = FootState.Resolve(msg.footholds[i]);
      }
    }
    else {
      resolved.footholds = []
    }

    return resolved;
    }
};

module.exports = FootPlanDiscrete;
