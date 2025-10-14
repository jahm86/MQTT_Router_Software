
#ifndef D_I_ERRORS_H
#define D_I_ERRORS_H

#include <cstdint>

struct DIError {
  enum ErrCode : int8_t {
    SUCCESS               =  0,
    FILE_DES_ERROR        = INT8_MIN,
    UNEXPECTED_JSON_TYPE  ,
    TYPE_NOT_INCLUDED     ,
    TYPE_NOT_EXIST        ,
    NAME_NOT_INCLUDED     ,
    SAME_SOURCE_NAMES     ,
    SAME_SRC_NODE_NAMES   ,
    SOURCE_NOT_FOUND      ,
    NODE_NOT_FOUND        ,
    PARAM_NOT_INCLUDED    ,
    PARAM_WORNG_TYPE      ,
    PARAM_OUT_BOUNDS      ,
    WRONG_ENUM            ,
    WRONG_CMD             ,
    PAUSED                ,
    BUS_REQ_ERROR         ,
    BUS_WR_DENIED         ,
    BUS_RD_DENIED         ,
    UNKNOWN               // Any other error
  };

  static const char* ErrStr(ErrCode code) {
    switch (code) {
      case ErrCode::SUCCESS:
        return "Success!";
      case ErrCode::FILE_DES_ERROR:
        return "File deserialization error";
      case ErrCode::UNEXPECTED_JSON_TYPE:
        return "Unexpected JSON type";
      case ErrCode::TYPE_NOT_INCLUDED:
        return "Object type not included";
      case ErrCode::TYPE_NOT_EXIST:
        return "Object type does not exists";
      case ErrCode::NAME_NOT_INCLUDED:
        return "Object name not included";
      case ErrCode::SAME_SOURCE_NAMES:
        return "Source names match";
      case ErrCode::SAME_SRC_NODE_NAMES:
        return "Node names match inside same source";
      case ErrCode::SOURCE_NOT_FOUND:
        return "Source not found";
      case ErrCode::NODE_NOT_FOUND:
        return "Node not found";
      case ErrCode::PARAM_NOT_INCLUDED:
        return "Required parameter not included";
      case ErrCode::PARAM_WORNG_TYPE:
        return "Wrong parameter type";
      case ErrCode::PARAM_OUT_BOUNDS:
        return "Parameter out of bounds";
      case ErrCode::WRONG_ENUM:
        return "Wrong enumerator";
      case ErrCode::WRONG_CMD:
        return "Wrong command";
      case ErrCode::PAUSED:
        return "System paused";
      case ErrCode::BUS_REQ_ERROR:
        return "Bus request error";
      case ErrCode::BUS_WR_DENIED:
        return "Bus write access denied";
      case ErrCode::BUS_RD_DENIED:
        return "Bus read access denied";
      default:
        return "Unknown error";
    }
  }
};


#endif // D_I_ERRORS_H
