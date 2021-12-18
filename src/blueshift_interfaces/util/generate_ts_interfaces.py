import sys
import pathlib

from rosidl_parser.parser import parse_idl_file

from rosidl_parser.definition import Action
from rosidl_parser.definition import Array
from rosidl_parser.definition import BasicType
from rosidl_parser.definition import BoundedSequence
from rosidl_parser.definition import BoundedString
from rosidl_parser.definition import BoundedWString
from rosidl_parser.definition import IdlLocator
from rosidl_parser.definition import Include
from rosidl_parser.definition import Message
from rosidl_parser.definition import NamespacedType
from rosidl_parser.definition import Service
from rosidl_parser.definition import UnboundedSequence
from rosidl_parser.definition import UnboundedString
from rosidl_parser.definition import UnboundedWString

basic_type_number_list = ["float", "float32", "float64", "double", "int8", "uint8", "int16", "uint16", "int32", "uint32", "int64", "uint64", "long double", "octet"]
basic_type_bool_list = ["bool", "boolean"]
basic_type_string_list = ["char", "wchar"]

# test command
# python3 util/generate_ts_interfaces.py /home/parallels/blueshift-ros/src/blueshift_interfaces/build/blueshift_interfaces blueshift_interfaces msg/Num.msg

def main(args=None):
    build_location = sys.argv[1]
    package_name = sys.argv[2]
    interface_list = sys.argv[3:]

    for interface in interface_list:
        interface_name, interface_type = interface.split(".")    

        interface_ast = parse_idl_file(
            IdlLocator(
                build_location, pathlib.Path('rosidl_adapter') / package_name / (interface_name + ".idl")
            )
        )
        generateTSInterface(interface_ast)

def generateTSInterface(ast):
    includes = ast.content.get_elements_of_type(Include)
    for include in includes:
        print(include.locator)
    
    messages = ast.content.get_elements_of_type(Message)
    for message in messages:
        print("  ", message.structure.namespaced_type.name, message.structure.namespaced_type.namespaces)
        for prop in message.structure.members:
            ts_type = getTSType(prop.type, prop)
            #print("    ", prop.name, prop.type)
            print(ts_type["name"]+":", ts_type["ts_type"]+",")
        for const in message.constants:
            ts_type = getTSType(const.type, const, const=True)
            print(ts_type["name"]+":", ts_type["value"]+",")
            #print("    ", const.name, const.type.typename, const.value)

def getTSType(T, prop=None, named=True, const=False):
    type_data = {
        "type": T,
        "ts_type": "",
        "value": ""
    }

    if prop and named:
        type_data['name'] = prop.name

    # change this to a switch statement once python 3.10 is supported
    if isinstance(T, BasicType):
        if T.typename in basic_type_number_list:
            type_data["ts_type"] = "number"
            if const:
                type_data["value"] = str(prop.value)
        elif T.typename in basic_type_bool_list:
            type_data["ts_type"] = "boolean"
            if const:
                type_data["value"] = str(prop.value).lower()
        elif T.typename in basic_type_string_list:
            type_data["ts_type"] = "string"
            if const:
                type_data["value"] = "'" + prop.value + "'"
        else:
            raise TypeError(f"Type {T} {T.typename} is not currently supported, please add a typescript definition")
    elif isinstance(T, UnboundedString) or isinstance(T, UnboundedWString) or isinstance(T, BoundedString) or isinstance(T, BoundedWString):
        # There is no clean way to limit string length in typescript
        # and it doesn't care about different character types
        # so we treat all of them a single "string" type
        type_data["ts_type"] = "string"
        if const:
            type_data["value"] = "'" + prop.value + "'"
    elif isinstance(T, UnboundedSequence):
        type_data["ts_type"] = f"Array<{ getTSType(T.value_type, named=False) }>"
    elif isinstance(T, BoundedSequence):
        type_data["ts_type"] = f"FixedSizeArray<{ T.maximum_size }, { getTSType(T.value_type, named=False) }>"
    elif isinstance(T, Array):
        type_data["ts_type"] = f"FixedSizeArray<{ T.size }, { getTSType(T.value_type, named=False) }>"
    elif isinstance(T, NamespacedType):
        pass
    else:
        raise TypeError(f"Type {T} is not currently supported, please add a typescript definition")

    if not named:
        return type_data["ts_type"]

    return type_data
if __name__ == '__main__':
    main()