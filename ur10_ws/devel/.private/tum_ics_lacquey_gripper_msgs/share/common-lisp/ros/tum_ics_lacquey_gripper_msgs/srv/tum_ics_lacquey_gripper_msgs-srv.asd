
(cl:in-package :asdf)

(defsystem "tum_ics_lacquey_gripper_msgs-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "getGripperState" :depends-on ("_package_getGripperState"))
    (:file "_package_getGripperState" :depends-on ("_package"))
    (:file "setGripperState" :depends-on ("_package_setGripperState"))
    (:file "_package_setGripperState" :depends-on ("_package"))
  ))