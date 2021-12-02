// 文字列をUTF8の16進文字列に変換
function string_to_utf8_hex_string(text) {
  const bytes1 = string_to_utf8_bytes(text);
  const hex_str1 = bytes_to_hex_string(bytes1);
  return hex_str1;
}
 
// UTF8の16進文字列を文字列に変換
function utf8_hex_string_to_string(hex_str1) {
  const bytes2 = hex_string_to_bytes(hex_str1);
  const str2 = utf8_bytes_to_string(bytes2);
  return str2;
}
 
// 文字列をUTF8のバイト配列に変換
function string_to_utf8_bytes(text) {
  let result = [];
  if (text == null)
    return result;
  for (let i = 0; i < text.length; i++) {
    let c = text.charCodeAt(i);
    if (c <= 0x7f) {
      result.push(c);
    } else if (c <= 0x07ff) {
      result.push(((c >> 6) & 0x1F) | 0xC0);
      result.push((c & 0x3F) | 0x80);
    } else {
      result.push(((c >> 12) & 0x0F) | 0xE0);
      result.push(((c >> 6) & 0x3F) | 0x80);
      result.push((c & 0x3F) | 0x80);
    }
  }
  return result;
}
 
// バイト値を16進文字列に変換
function byte_to_hex(byte_num) {
  let digits = (byte_num).toString(16);
  if (byte_num < 16) return '0' + digits;
  return digits;
}
 
// バイト配列を16進文字列に変換
function bytes_to_hex_string(bytes) {
  let result = "";
 
  for (let i = 0; i < bytes.length; i++) {
    result += byte_to_hex(bytes[i]);
  }
  return result;
}
 
// 16進文字列をバイト値に変換
function hex_to_byte(hex_str) {
  return parseInt(hex_str, 16);
}
 
// バイト配列を16進文字列に変換
function hex_string_to_bytes(hex_str) {
  let result = [];
 
  for (let i = 0; i < hex_str.length; i += 2) {
    result.push(hex_to_byte(hex_str.substr(i, 2)));
  }
  return result;
}
 
// UTF8のバイト配列を文字列に変換
function utf8_bytes_to_string(arr) {
  if (arr == null)
    return null;
  let result = "";
  let i;
  while (i = arr.shift()) {
    if (i <= 0x7f) {
      result += String.fromCharCode(i);
    } else if (i <= 0xdf) {
      let c = ((i & 0x1f) << 6);
      c += arr.shift() & 0x3f;
      result += String.fromCharCode(c);
    } else if (i <= 0xe0) {
      let c = ((arr.shift() & 0x1f) << 6) | 0x0800;
      c += arr.shift() & 0x3f;
      result += String.fromCharCode(c);
    } else {
      let c = ((i & 0x0f) << 12);
      c += (arr.shift() & 0x3f) << 6;
      c += arr.shift() & 0x3f;
      result += String.fromCharCode(c);
    }
  }
  return result;
}
 
//4byte分の整数を32bitの配列に分解
function byte4to32bit(byte0,byte1,byte2,byte3){
  let byteList = [byte0,byte1,byte2,byte3];
  let intList=[];
  for(let j=0;j<4;j++){
    for(let i=0;i<8;i++){
      intList.push( (byteList[j] >> i ) & 0x01);
    }
  }
  return intList;
}
 
//16進文字列をbyte毎の数値配列に変換
function hexstr2intList(hexstr,num_) {
  let intList = [];
  for (let i = 0; i < num_; i = i + 2) {
    intList.push(parseInt(hexstr.slice(i, i + 2), 16));
  }
  return intList;
}

//ArrayBufferを文字列に変換
function buffer_to_string(buf) {
  return String.fromCharCode.apply("", new Uint8Array(buf))
}


export {
  string_to_utf8_hex_string,
  utf8_hex_string_to_string,
  string_to_utf8_bytes, 
  byte_to_hex, 
  bytes_to_hex_string,
  hex_to_byte,
  hex_string_to_bytes,
  utf8_bytes_to_string,
  byte4to32bit,
  hexstr2intList,
  buffer_to_string
};