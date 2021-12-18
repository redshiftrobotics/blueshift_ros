import sys
import pathlib
import subprocess

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

output = ""

name_mapping = {}

build_location = sys.argv[1]
package_name = sys.argv[2]
interface_list = sys.argv[3:]

def main(args=None):
    global output
    global name_mapping

    output += "interface ROSMessageBase {}\n"
    output += "\n"
    output += """// https://mstn.github.io/2018/06/08/fixed-size-arrays-in-typescript/
type FixedSizeArray<N extends number, T> = N extends 0 ? never[] : {
    0: T;
    length: N;
} & ReadonlyArray<T>;\n"""

    for interface in interface_list:
        interface_name, interface_type = interface.split(".")    

        interface_ast = parse_idl_file(
            IdlLocator(
                build_location, pathlib.Path('rosidl_adapter') / package_name / (interface_name + ".idl")
            )
        )
        generateTSInterface(interface_ast)

    output += "\n"
    output += "export type ROSMessageStrings = " + "|".join(["'"+key+"'" for key in name_mapping.keys()]) + ";\n"

    output += "\n"
    output += "export type ROSMessagesTypeTSDefinitions = {" + ",".join(["'"+key+"'" + ": " + name_mapping[key]["interface"] for key in name_mapping.keys()]) + "}\n"

    output += "\n"
    output += "export let ROSMessageFactories = {" + ",".join(["'"+key+"'" + ": " + name_mapping[key]["factory"] for key in name_mapping.keys()]) + "}\n"
    
    output += "\n"
    output += """// https://fettblog.eu/typescript-type-maps/
// https://blog.rsuter.com/how-to-instantiate-a-generic-type-in-typescript/
export type ROSMessage<T extends ROSMessageStrings> =
    T extends keyof ROSMessagesTypeTSDefinitions ? ROSMessagesTypeTSDefinitions[T] :
    ROSMessageBase;"""

    with open("out.ts", "w") as f:
        f.write(output)

def generateTSInterface(ast):
    global output

    includes = ast.content.get_elements_of_type(Include)
    for include in includes:
        pkg, folder, idl = include.locator.split("/")
        ros_find_package, err = subprocess.Popen(["ros2", "pkg", "prefix", pkg], stdout=subprocess.PIPE).communicate()

        if ros_find_package:
            base_folder = pathlib.Path(ros_find_package.decode().strip()) / 'share' / pkg / folder
        else:
            base_folder = pathlib.Path(build_location) / 'rosidl_adapter' / pkg / folder

        include_interface_ast = parse_idl_file(IdlLocator(base_folder, idl))
        generateTSInterface(include_interface_ast)

    messages = ast.content.get_elements_of_type(Message)
    for message in messages:
        tsName, tsFactory, rosInterfaceString = getName(message.structure.namespaced_type)
        
        interface_str = f"export interface {tsName} extends ROSMessageBase {{"
        factory_str = f"function {tsFactory}(): {tsName} {{ return {{"
        
        name_mapping[rosInterfaceString] = {
            "interface": tsName,
            "factory": tsFactory
        }

        for prop in message.structure.members:
            ts_type = getTSType(prop.type, prop)
            interface_str += ts_type["name"] + ": " + ts_type["ts_type"] + ";"
            factory_str += ts_type["name"] + ": " + getDefaultValue(ts_type) + ","
        for const in message.constants:
            ts_type = getTSType(const.type, const, const=True)
            interface_str += ts_type["name"] + ": " + ts_type["value"] + ";"
            factory_str += ts_type["name"] + ": " + ts_type["value"] + ","
        
        interface_str += "}"
        factory_str = factory_str[:-1] + "};}"

        output += "\n"
        output += interface_str + "\n"
        output += "\n"
        output += factory_str + "\n"

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
        type_data["length"] = T.maximum_size
        type_data["ts_element_type"] = getTSType(T.value_type, named=False)
    elif isinstance(T, Array):
        type_data["ts_type"] = f"FixedSizeArray<{ T.size }, { getTSType(T.value_type, named=False) }>"
        type_data["length"] = T.size
        type_data["ts_element_type"] = getTSType(T.value_type, named=False)
    elif isinstance(T, NamespacedType):
        type_data["ts_type"] = getName(T)[0]
    else:
        raise TypeError(f"Type {T} is not currently supported, please add a typescript definition")

    if not named:
        return type_data["ts_type"]

    return type_data

def getName(namespace):
    return f"{namespace.namespaces[0]}__{namespace.name}__{namespace.namespaces[1]}", \
        f"{namespace.namespaces[0]}__{namespace.name}__{namespace.namespaces[1]}__Factory", \
        f"{namespace.namespaces[0]}/{namespace.name}"

default_value_dict = {
    "number": "0",
    "boolean": "false",
    "string": "''"
}

def getDefaultValue(type_data):
    if type_data['ts_type'] in default_value_dict:
        return default_value_dict[type_data['ts_type']]
    elif type_data['ts_type'].startswith("Array"):
        return "[]"
    elif type_data['ts_type'].startswith("FixedSizeArray"):
        return "[" + ((default_value_dict[type_data['ts_element_type']] + ",") * type_data['length'])[:-1] + "]"
    elif type_data['ts_type'] in [name_mapping[key]["interface"] for key in name_mapping.keys()]:
        return getName(type_data["type"])[1] + "()"
    else:
        raise TypeError(f"There is no default value for type {type_data['ts_type']}. Please add one")

if __name__ == '__main__':
    main()