// Auto-generated. Do not edit!

// (in-package tum_ics_lacquey_gripper_msgs.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class setGripperStateRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.newState = null;
    }
    else {
      if (initObj.hasOwnProperty('newState')) {
        this.newState = initObj.newState
      }
      else {
        this.newState = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type setGripperStateRequest
    // Serialize message field [newState]
    bufferOffset = _serializer.string(obj.newState, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type setGripperStateRequest
    let len;
    let data = new setGripperStateRequest(null);
    // Deserialize message field [newState]
    data.newState = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.newState);
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'tum_ics_lacquey_gripper_msgs/setGripperStateRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '99fe01dfe585dc3e99fcbbc1365932c3';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # set gripper state
    string newState
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new setGripperStateRequest(null);
    if (msg.newState !== undefined) {
      resolved.newState = msg.newState;
    }
    else {
      resolved.newState = ''
    }

    return resolved;
    }
};

class setGripperStateResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.ok = null;
    }
    else {
      if (initObj.hasOwnProperty('ok')) {
        this.ok = initObj.ok
      }
      else {
        this.ok = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type setGripperStateResponse
    // Serialize message field [ok]
    bufferOffset = _serializer.bool(obj.ok, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type setGripperStateResponse
    let len;
    let data = new setGripperStateResponse(null);
    // Deserialize message field [ok]
    data.ok = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'tum_ics_lacquey_gripper_msgs/setGripperStateResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '6f6da3883749771fac40d6deb24a8c02';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool ok
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new setGripperStateResponse(null);
    if (msg.ok !== undefined) {
      resolved.ok = msg.ok;
    }
    else {
      resolved.ok = false
    }

    return resolved;
    }
};

module.exports = {
  Request: setGripperStateRequest,
  Response: setGripperStateResponse,
  md5sum() { return '8223e23338c4551db84e86bb33c66825'; },
  datatype() { return 'tum_ics_lacquey_gripper_msgs/setGripperState'; }
};
