// Auto-generated. Do not edit!

// (in-package mujoco_ros_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class SimstatusM2C {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.time = null;
      this.sim_paused = null;
      this.sim_reset = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('time')) {
        this.time = initObj.time
      }
      else {
        this.time = 0.0;
      }
      if (initObj.hasOwnProperty('sim_paused')) {
        this.sim_paused = initObj.sim_paused
      }
      else {
        this.sim_paused = false;
      }
      if (initObj.hasOwnProperty('sim_reset')) {
        this.sim_reset = initObj.sim_reset
      }
      else {
        this.sim_reset = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SimstatusM2C
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [time]
    bufferOffset = _serializer.float64(obj.time, buffer, bufferOffset);
    // Serialize message field [sim_paused]
    bufferOffset = _serializer.bool(obj.sim_paused, buffer, bufferOffset);
    // Serialize message field [sim_reset]
    bufferOffset = _serializer.bool(obj.sim_reset, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SimstatusM2C
    let len;
    let data = new SimstatusM2C(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [time]
    data.time = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [sim_paused]
    data.sim_paused = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [sim_reset]
    data.sim_reset = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 10;
  }

  static datatype() {
    // Returns string type for a message object
    return 'mujoco_ros_msgs/SimstatusM2C';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '59140af914df0ac28c7df06d452568bd';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    #MUJOCO -> CONTROLLER SIM STATUS COMMANDER
    
    
    Header header
    float64 time
    
    bool sim_paused
    bool sim_reset
    
    
    
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
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SimstatusM2C(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.time !== undefined) {
      resolved.time = msg.time;
    }
    else {
      resolved.time = 0.0
    }

    if (msg.sim_paused !== undefined) {
      resolved.sim_paused = msg.sim_paused;
    }
    else {
      resolved.sim_paused = false
    }

    if (msg.sim_reset !== undefined) {
      resolved.sim_reset = msg.sim_reset;
    }
    else {
      resolved.sim_reset = false
    }

    return resolved;
    }
};

module.exports = SimstatusM2C;
