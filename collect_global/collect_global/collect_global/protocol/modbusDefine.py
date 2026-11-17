import  os
import  sys
from ..common import define as define
#枚举定义
from enum import unique, Enum, auto
from ..common import commonTools as tools
from ..common.logManager import output, outputMode,enum_Error
from typing import cast
#导入pymodbus库,如果失败自动安装,现在确定版本
try:
    from pymodbus.client.sync import ModbusTcpClient
    from pymodbus.exceptions import ModbusException
    from pymodbus.pdu import ExceptionResponse
    from pymodbus.exceptions import ConnectionException
    from pymodbus.utilities import make_byte_string
    from pymodbus.payload import BinaryPayloadDecoder
    from pymodbus.payload import BinaryPayloadBuilder
    from pymodbus.constants import Endian
except ImportError:
    #if sys.version.find('3.7.') != -1:
    os.system("pip install pyserial==3.5.0")
    os.system("pip install pymodbus==2.5.3 -i https://mirrors.aliyun.com/pypi/simple/")
    #else:    
    #    os.system("pip install pyserial")
    #    os.system("pip install pymodbus")

@unique#枚举类修饰符，唯一检查
class enum_Modbus_FunctionCode(Enum):
    NONE_FUNCTION               = 0x00 #没有定义
    READ_COILS                  = 0x01 #读取输出线圈(可读可写)
    READ_Discrete_Input         = 0x02 #读取输入线圈(只读)
    READ_HOLDING_REGISTERS      = 0x03 #读取保持寄存器(可读可写)
    READ_INPUT_REGISTERS        = 0x04 #读取输入寄存器(只读)
    WRITE_SINGLE_COIL           = 0x05 #写单个线圈
    WRITE_HOLDING_REGISTERS     = 0x06 #写保存寄存器
    WRITE_MULTIPLE_COILS        = 0x0F #写多线圈
    WRITE_MULTIPLE_REGISTERS    = 0x10 #写多寄存器\
#从全部数据类型中抽取出modbusw支持的类型
@unique#枚举类修饰符，唯一检查
class enum_Modbus_DataType(Enum):
    NONE_DATA   = define.Enum_Data_type.none.value      #不有定义
    UINT16      = define.Enum_Data_type.uint.value      #无符号16位
    INT16       = define.Enum_Data_type.int.value       #有符号16位
    UINT32      = define.Enum_Data_type.udint.value     #无符号32位
    INT32       = define.Enum_Data_type.dint.value      #有符号32位
    FLOAT       = define.Enum_Data_type.real.value      #浮点数
    DOUBLE      = define.Enum_Data_type.lreal.value     #浮点数
    UINT64      = define.Enum_Data_type.ulint.value     #无符号64位
    INT64       = define.Enum_Data_type.lint.value      #有符号64位
    CHAR        = define.Enum_Data_type.string.value    #字符
    WCHAR       = define.Enum_Data_type.wstring.value   #宽字符
    INT_BCD     = define.Enum_Data_type.int_BCD.value   #BCD码
    INT32_BCD   = define.Enum_Data_type.dint_BCD.value  #BCD码

modbusTypeLen = {define.Enum_Data_type.uint.value :1,
    define.Enum_Data_type.int.value:1,
    define.Enum_Data_type.uint.value:1,
    define.Enum_Data_type.udint.value:2,
    define.Enum_Data_type.dint.value:2,
    define.Enum_Data_type.real.value:2,
    define.Enum_Data_type.lreal.value:4,
    define.Enum_Data_type.ulint.value:4, #无符号64位
    define.Enum_Data_type.lint.value:4, #有符号64位
    define.Enum_Data_type.int_BCD.value:1,
    define.Enum_Data_type.dint_BCD.value:2
    }

class ModbusStringParser:
    def __init__(self, byte_order='big', word_order='big'):
        """
        初始化解析器
        
        参数:
            word_order: 'big'或'little' - 字序（寄存器顺序）
            byte_order: 'big'或'little' - 字节序（寄存器内字节顺序）
        """
        self.word_order = word_order
        self.byte_order = byte_order
    
    def parse_ascii(self, registers, start=0, length=None, null_terminated=True):
        """
        解析ASCII字符串
        
        参数:
            registers: 寄存器值列表
            start: 起始寄存器索引
            length: 期望的字符数
            null_terminated: 是否以NULL结尾
        """
        if self.word_order == 'little':
            # 反转寄存器顺序
            registers = registers.copy()
            
        result = ""
        char_count = 0
        
        for reg in registers[start:]:
            if length is not None and char_count >= length:
                break
            
            # 获取字节
            if self.byte_order == 'big':
                bytes_data = [(reg >> 8) & 0xFF, reg & 0xFF]
            else:
                bytes_data = [reg & 0xFF, (reg >> 8) & 0xFF]
            
            for byte_val in bytes_data:
                if length is not None and char_count >= length:
                    break
                
                if byte_val == 0 and null_terminated:
                    return result
                
                if byte_val != 0 or not null_terminated:
                    try:
                        result += chr(byte_val)
                        char_count += 1
                    except:
                        # 非ASCII字符
                        result += '?'
        
        return result
    
    def parse_utf16(self, registers, start=0, length=None):
        """
        解析UTF-16字符串
        """
        if self.word_order == 'little':
            registers = registers[start:].copy()
        else:
            registers = registers[start:]
        
        result = ""
        char_count = 0
        
        for reg in registers:
            if length is not None and char_count >= length:
                break
            
            # 直接转换寄存器值为字符
            try:
                # 假设每个寄存器是一个UTF-16字符
                result += chr(reg)
                char_count += 1
            except:
                result += '?'
        
        return result
    
    def parse_with_encoding(self, registers, encoding='ascii', start=0, length=None):
        """
        使用指定编码解析
        
        参数:
            encoding: 'ascii', 'utf-8', 'gbk', 'latin-1'等
        """
        # 将寄存器转换为字节数组
        byte_array = bytearray()
        
        if self.byte_order == 'big':
            if len(registers) >=2:
                for i in range(0, len(registers), 2):
                    byte_array.append(registers[i+1])
                    byte_array.append(registers[i])
                if len(registers) % 2 != 0:
                    byte_array.append(registers[-1])
        else:
            byte_array = registers[0:]
        
        # 截取指定长度
        if length is not None and length * 2 <= len(byte_array):
            byte_array = byte_array[:length * 2]
        
        try:
            # 查找NULL终止符
            if b'\x00' in byte_array:
                null_pos = byte_array.index(b'\x00')
                byte_array = byte_array[:null_pos]
            
            # 解码
            return byte_array.decode(encoding, errors='replace')
        except:
            return byte_array.decode('latin-1', errors='replace')
class modbus_tools:
    #生成编码实例,如果失败返回None,使用时需要判断,正常情况下不会是None
    def decode_instance(byte_order,word_order,registerData):
        decoder = None
        try:
            ebyteOrder = Endian.Big if byte_order == define.byte_order[0] else Endian.Little
            ewordorder = Endian.Big if word_order == define.byte_order[0] else Endian.Little
            decoder = BinaryPayloadDecoder.fromRegisters(registerData,byteorder= ebyteOrder,wordorder=ewordorder)
        except Exception as e:
            output(f'[异常]:[modbus_tools.decode_instance]:{e}',outputMode.srceen,enum_Error.localErr)
            decorder = None
        return decoder
    #编码值写入编码实例中,
    def decode_value(decoder:BinaryPayloadDecoder,name,datalen,dataType,byteOrder='big',wordOrder='big')->[bool,str]:
        bRes = True
        strVal = ""
        try:
            if dataType == enum_Modbus_DataType.UINT16.value:         #无符号16位
                strVal = decoder.decode_16bit_uint()
            elif dataType == enum_Modbus_DataType.INT16.value:        #有符号16位
                strVal = decoder.decode_16bit_int()
            elif dataType == enum_Modbus_DataType.UINT32.value:       #无符号32位
                strVal = decoder.decode_32bit_uint()
            elif dataType == enum_Modbus_DataType.INT32.value:        #有符号32位
                strVal = decoder.decode_32bit_int()
            elif dataType == enum_Modbus_DataType.FLOAT.value:        #浮点数
                strVal = decoder.decode_32bit_float()
            elif dataType == enum_Modbus_DataType.DOUBLE.value:       #双精度浮点数
                strVal = decoder.decode_64bit_float()
            elif dataType == enum_Modbus_DataType.UINT64.value:       #无符号64位
                strVal = decoder.decode_64bit_uint()
            elif dataType == enum_Modbus_DataType.INT64.value:        #有符号64位
                strVal = decoder.decode_64bit_int()
            elif dataType == enum_Modbus_DataType.CHAR.value:         #字符
                raw_bytes = decoder.decode_string(datalen)
                parser = ModbusStringParser(byteOrder,wordOrder)
                strVal = parser.parse_with_encoding(raw_bytes)
            elif dataType == enum_Modbus_DataType.WCHAR.value:        #宽字符
                raw_bytes = decoder.decode_string(datalen)
                parser = ModbusStringParser(byteOrder,wordOrder)
                strVal = parser.parse_with_encoding(raw_bytes,encoding='utf-8')
            elif dataType == enum_Modbus_DataType.INT_BCD.value:      #2字节BCD
                raw_bytes = decoder.decode_16bit_uint()
                strVal = tools.BCD_Manager.bcd_to_int_complete(raw_bytes.to_bytes(2, byteorder='big'))
            elif dataType == enum_Modbus_DataType.INT32_BCD.value:    #4字节BCD
                raw_bytes = decoder.decode_string(datalen*2)
                strVal = tools.BCD_Manager.bcd_to_int_complete(raw_bytes)
            else:
                output(f'[异常]:[modbus_tools.decode_value]:解析类型错误:{name}-({dataType})',outputMode.srceen,enum_Error.localErr)
                bRes = False
        except Exception as e:
            output(f'[异常]:[modbus_tools.decode_value]:{e}',outputMode.saveLog,enum_Error.localErr)
            bRes = False
        return bRes,str(strVal)

    def encode_value(encoder:BinaryPayloadBuilder,dataType,data,byteOrder='big',wordOrder='big'):
        if dataType == enum_Modbus_DataType.UINT16.value:         #无符号16位
            encoder.add_16bit_uint(tools.dataConvertManager.data2int(data))
        elif dataType == enum_Modbus_DataType.INT16.value:        #有符号16位
            encoder.add_16bit_int(tools.dataConvertManager.data2int(data))
        elif dataType == enum_Modbus_DataType.UINT32.value:       #无符号32位
            encoder.add_32bit_uint(tools.dataConvertManager.data2int(data))
        elif dataType == enum_Modbus_DataType.INT32.value:        #有符号32位
            encoder.add_32bit_int(tools.dataConvertManager.data2int(data))
        elif dataType == enum_Modbus_DataType.FLOAT.value:        #浮点数
            encoder.add_32bit_float(tools.dataConvertManager.data2float(data))
        elif dataType == enum_Modbus_DataType.DOUBLE.value:       #双精度浮点数
            encoder.add_64bit_float(tools.dataConvertManager.data2float(data))
        elif dataType == enum_Modbus_DataType.UINT64.value:       #无符号64位
            encoder.add_64bit_uint(tools.dataConvertManager.data2int(data))
        elif dataType == enum_Modbus_DataType.INT64.value:        #有符号64位
            encoder.add_64bit_int(tools.dataConvertManager.data2int(data))
        elif dataType == enum_Modbus_DataType.CHAR.value:         #字符
            byteVal = data.encode()
            parser = ModbusStringParser(byteOrder,wordOrder)
            strVal = parser.parse_with_encoding(byteVal)
            encoder.add_string(strVal)
        elif dataType == enum_Modbus_DataType.WCHAR.value:        #宽字符
            encoder.add_string(data)
        elif dataType == enum_Modbus_DataType.INT_BCD.value:      #2字节BCD
            encoder.add_16bit_uint(int.from_bytes(tools.BCD_Manager.int_to_bcd_16bit(tools.dataConvertManager.data2int(data)),byteorder =byteOrder))
        elif dataType == enum_Modbus_DataType.INT32_BCD.value:    #4字节BCD
            encoder.add_32bit_uint(int.from_bytes(tools.BCD_Manager.int_to_bcd_32bit(tools.dataConvertManager.data2int(data)),byteorder =byteOrder))

    def encode_instance(byte_order,word_order,preData):
        encoder = None
        ebyteOrder = Endian.Big if byte_order == define.byte_order[0] else Endian.Little
        ewordorder = Endian.Big if word_order == define.byte_order[0] else Endian.Little
        
        encoder = BinaryPayloadBuilder(
            preData,
            byteorder=ebyteOrder,
            wordorder=ewordorder
        )
        
        return encoder


