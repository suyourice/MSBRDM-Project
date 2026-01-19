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

class getGripperStateRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type getGripperStateRequest
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type getGripperStateRequest
    let len;
    let data = new getGripperStateRequest(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'tum_ics_lacquey_gripper_msgs/getGripperStateRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd41d8cd98f00b204e9800998ecf8427e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new getGripperStateRequest(null);
    return resolved;
    }
};

class getGripperStateResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.states = null;
      this.currentState = null;
    }
    else {
      if (initObj.hasOwnProperty('states')) {
        this.states = initObj.states
      }
      else {
        this.states = [];
      }
      if (initObj.hasOwnProperty('currentState')) {
        this.currentState = initObj.currentState
      }
      else {
        this.currentState = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type getGripperStateResponse
    // Serialize message field [states]
    bufferOffset = _arraySerializer.string(obj.states, buffer, bufferOffset, null);
    // Serialize message field [currentState]
    bufferOffset = _serializer.string(obj.currentState, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type getGripperStateResponse
    let len;
    let data = new getGripperStateResponse(null);
    // Deserialize message field [states]
    data.states = _arrayDeserializer.string(buffer, bufferOffset, null)
    // Deserialize message field [currentState]
    data.currentState = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    object.states.forEach((val) => {
      length += 4 + _getByteLength(val);
    });
    length += _getByteLength(object.currentState);
    return length + 8;
  }

  static datatype() {
    // Returns string type for a service object
    return 'tum_ics_lacquey_gripper_msgs/getGripperStateResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'ff0be24d823aa1cfde92641488f9e02a';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string[] states
    string currentState
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new getGripperStateResponse(null);
    if (msg.states !== undefined) {
      resolved.states = msg.states;
    }
    else {
      resolved.states = []
    }

    if (msg.currentState !== undefined) {
      resolved.currentState = msg.currentState;
    }
    else {
      resolved.currentState = ''
    }

    return resolved;
    }
};

module.exports = {
  Request: getGripperStateRequest,
  Response: getGripperStateResponse,
  md5sum() { return 'ff0be24d823aa1cfde92641488f9e02a'; },
  datatype() { return 'tum_ics_lacquey_gripper_msgs/getGripperState'; }
};
