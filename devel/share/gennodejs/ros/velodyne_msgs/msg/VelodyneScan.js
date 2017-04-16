// Auto-generated. Do not edit!

// (in-package velodyne_msgs.msg)


"use strict";

let _serializer = require('../base_serialize.js');
let _deserializer = require('../base_deserialize.js');
let _finder = require('../find.js');
let VelodynePacket = require('./VelodynePacket.js');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class VelodyneScan {
  constructor() {
    this.header = new std_msgs.msg.Header();
    this.packets = [];
  }

  static serialize(obj, bufferInfo) {
    // Serializes a message object of type VelodyneScan
    // Serialize message field [header]
    bufferInfo = std_msgs.msg.Header.serialize(obj.header, bufferInfo);
    // Serialize the length for message field [packets]
    bufferInfo = _serializer.uint32(obj.packets.length, bufferInfo);
    // Serialize message field [packets]
    obj.packets.forEach((val) => {
      bufferInfo = VelodynePacket.serialize(val, bufferInfo);
    });
    return bufferInfo;
  }

  static deserialize(buffer) {
    //deserializes a message object of type VelodyneScan
    let tmp;
    let len;
    let data = new VelodyneScan();
    // Deserialize message field [header]
    tmp = std_msgs.msg.Header.deserialize(buffer);
    data.header = tmp.data;
    buffer = tmp.buffer;
    // Deserialize array length for message field [packets]
    tmp = _deserializer.uint32(buffer);
    len = tmp.data;
    buffer = tmp.buffer;
    // Deserialize message field [packets]
    data.packets = new Array(len);
    for (let i = 0; i < len; ++i) {
      tmp = VelodynePacket.deserialize(buffer);
      data.packets[i] = tmp.data;
      buffer = tmp.buffer;
    }
    return {
      data: data,
      buffer: buffer
    }
  }

  static datatype() {
    // Returns string type for a message object
    return 'velodyne_msgs/VelodyneScan';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '50804fc9533a0e579e6322c04ae70566';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Velodyne LIDAR scan packets.
    
    Header           header         # standard ROS message header
    VelodynePacket[] packets        # vector of raw packets
    
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
    # 0: no frame
    # 1: global frame
    string frame_id
    
    ================================================================================
    MSG: velodyne_msgs/VelodynePacket
    # Raw Velodyne LIDAR packet.
    
    time stamp              # packet timestamp
    uint8[1206] data        # packet contents
    
    
    `;
  }

};

module.exports = VelodyneScan;
