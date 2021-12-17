import sys
import os
import pathlib

from rosidl_parser.parser import parse_idl_file
from rosidl_parser.definition import IdlLocator
from rosidl_parser.definition import Message

def main(args=None):
    # print(sys.argv[1:])

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

        # sample parser code: https://github.com/ros2/rosidl/blob/master/rosidl_parser/test/test_parser.py
        print(interface_ast.content.get_elements_of_type(Message)[0].structure.members[0].name)


if __name__ == '__main__':
    main()