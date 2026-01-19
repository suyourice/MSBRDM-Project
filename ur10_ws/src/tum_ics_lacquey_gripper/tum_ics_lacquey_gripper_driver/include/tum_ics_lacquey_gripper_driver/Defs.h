#ifndef LACQUEY_GRIPPER_DRIVER_GRIPPER_DEFS_H
#define LACQUEY_GRIPPER_DRIVER_GRIPPER_DEFS_H

namespace tum_ics_lacquey_gripper_driver{
namespace Gripper{

enum State
{
    OPEN_STATE = 0,
    CLOSED_STATE
};

enum Type
{
    LEFT_GRIPPER = 0,
    RIGHT_GRIPPER,
    STANDARD_GRIPPER
};


}}

#endif // LACQUEY_GRIPPER_DRIVER_GRIPPER_DEFS_H
