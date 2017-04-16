// Auto-generated. Do not edit!

// (in-package velodyne_msgs.msg)


"use strict";

let _serializer = require('../base_serialize.js');
let _deserializer = require('../base_deserialize.js');
let _finder = require('../find.js');

//-----------------------------------------------------------

class VelodynePacket {
  constructor() {
    this.stamp = {secs: 0, nsecs: 0};
    this.data = new Array(1206).fill(0);
  }

  static serialize(obj, bufferInfo) {
    // Serializes a message object of type VelodynePacket
    // Serialize message field [stamp]
    bufferInfo = _serializer.time(obj.stamp, bufferInfo);
    // Serialize message field [data]
    bufferInfo.buffer.push(obj.data);
    bufferInfo.length += obj.data.length;
    return bufferInfo;
  }

  static deserialize(buffer) {
    //deserializes a message object of type VelodynePacket
    let tmp;
    let len;
    let data = new VelodynePacket();
    // Deserialize message field [stamp]
    tmp = _deserializer.time(buffer);
    data.stamp = tmp.data;
    buffer = tmp.buffer;
    len = 1206;
    // Deserialize message field [data]
    data.data = buffer.slice(0, len);
    buffer =  buffer.slice(len);
    return {
      data: data,
      buffer: buffer
    }
  }

  static datatype() {
    // Returns string type for a message object
    return 'velodyne_msgs/VelodynePacket';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'ae4f90a23256f44e82baa08dd45c3456';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Raw Velodyne LIDAR packet.
    
    time stamp              # packet timestamp
    uint8[1206] data        # packet contents
    
    
    `;
  }

};

module.exports = VelodynePacket;
