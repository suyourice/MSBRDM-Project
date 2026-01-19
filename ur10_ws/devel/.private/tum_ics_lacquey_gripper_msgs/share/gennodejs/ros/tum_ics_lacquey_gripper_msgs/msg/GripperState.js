// Auto-generated. Do not edit!

// (in-package tum_ics_lacquey_gripper_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class GripperState {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.state = null;
      this.stateId = null;
    }
    else {
      if (initObj.hasOwnProperty('state')) {
        this.state = initObj.state
      }
      else {
        this.state = '';
      }
      if (initObj.hasOwnProperty('stateId')) {
        this.stateId = initObj.stateId
      }
      else {
        this.stateId = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GripperState
    // Serialize message field [state]
    bufferOffset = _serializer.string(obj.state, buffer, bufferOffset);
    // Serialize message field [stateId]
    bufferOffset = _serializer.int64(obj.stateId, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GripperState
    let len;
    let data = new GripperState(null);
    // Deserialize message field [state]
    data.state = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [stateId]
    data.stateId = _deserializer.int64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.state);
    return length + 12;
  }

  static datatype() {
    // Returns string type for a message object
    return 'tum_ics_lacquey_gripper_msgs/GripperState';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '6806ba1ebf38d757b7a04e49116d3b97';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # gripper state
    string state
    int64 stateId
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GripperState(null);
    if (msg.state !== undefined) {
      resolved.state = msg.state;
    }
    else {
      resolved.state = ''
    }

    if (msg.stateId !== undefined) {
      resolved.stateId = msg.stateId;
    }
    else {
      resolved.stateId = 0
    }

    return resolved;
    }
};

module.exports = GripperState;
