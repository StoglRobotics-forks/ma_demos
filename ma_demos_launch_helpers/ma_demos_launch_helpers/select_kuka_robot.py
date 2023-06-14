kuka_kr3 = {
    "robot_description_package": "kuka_kr3_support",
    "robot_description_macro_file": "kr3r540_macro.xacro",
    "robot_name": "kuka_kr3r540",
}

kuka_kr5 = {
    "robot_description_package": "kuka_kr5_support",
    "robot_description_macro_file": "kr5_arc_macro.xacro",
    "robot_name": "kuka_kr5_arc",
}

kuka_kr16_2 = {
    "robot_description_package": "kuka_kr16_support",
    "robot_description_macro_file": "kr16_2_macro.xacro",
    "robot_name": "kuka_kr16_2",
}


def select_kuka_robot(robot_type):
    if robot_type == "kuka_kr3":
        return kuka_kr3
    elif robot_type == "kuka_kr5":
        return kuka_kr5
    elif robot_type == "kuka_kr16_2":
        return kuka_kr16_2
    else:
        raise (RuntimeError(f"Unknown robot_type:{robot_type}"))
