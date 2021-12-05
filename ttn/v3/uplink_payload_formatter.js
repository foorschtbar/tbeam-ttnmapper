function decodeUplink(input) {
  
    data = Decoder(input.bytes,input.fPort);
    return {
      data,
      warnings: [],
      errors: []
    };
  }
  
  function bytesToFloat(by) {
      var bits = by[3]<<24 | by[2]<<16 | by[1]<<8 | by[0];
      var sign = (bits>>>31 === 0) ? 1.0 : -1.0;
      var e = bits>>>23 & 0xff;
      var m = (e === 0) ? (bits & 0x7fffff)<<1 : (bits & 0x7fffff) | 0x800000;
      var f = sign * m * Math.pow(2, e - 150);
      return f;
  } 
  
  function bytesToInt(by) {
      f = by[0] | by[1]<<8 | by[2]<<16 | by[3]<<24;
      return f;
  } 
  
  var decoded = {};
      
  function Decoder(bytes, port) {
    // TTGO-Beam TTN Mapper
    if(port==1){
  
      decoded.latitude = ((bytes[0]<<16)>>>0) + ((bytes[1]<<8)>>>0) + bytes[2];
      decoded.latitude = (decoded.latitude / 16777215.0 * 180) - 90;
    
      decoded.longitude = ((bytes[3]<<16)>>>0) + ((bytes[4]<<8)>>>0) + bytes[5];
      decoded.longitude = (decoded.longitude / 16777215.0 * 360) - 180;
    
      var altValue = ((bytes[6]<<8)>>>0) + bytes[7];
      var sign = bytes[6] & (1 << 7);
      if(sign)
      {
          decoded.altitude = 0xFFFF0000 | altValue;
      }
      else
      {
          decoded.altitude = altValue;
      }
    
      decoded.hdop = bytes[8] / 10.0;
  
      return decoded;
      
    // Cubecell TTN Mapper  
    }else if (port==2){
  
      i = 0;
    
      decoded.latitude = bytesToFloat(bytes.slice(i,i+=4));
      decoded.longitude = bytesToFloat(bytes.slice(i,i+=4));
      decoded.altitude = bytesToFloat(bytes.slice(i,i+=4));
      decoded.course = bytesToFloat(bytes.slice(i,i+=4));
      decoded.speed = bytesToFloat(bytes.slice(i,i+=4));
      decoded.hdop = bytesToFloat(bytes.slice(i,i+=4));
    
      decoded.battery = ((bytes[i++] << 8) | bytes[i++]);
  
      return decoded;
  
    }
  }