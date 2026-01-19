;; Auto-generated. Do not edit!


(when (boundp 'tum_ics_lacquey_gripper_msgs::setGripperState)
  (if (not (find-package "TUM_ICS_LACQUEY_GRIPPER_MSGS"))
    (make-package "TUM_ICS_LACQUEY_GRIPPER_MSGS"))
  (shadow 'setGripperState (find-package "TUM_ICS_LACQUEY_GRIPPER_MSGS")))
(unless (find-package "TUM_ICS_LACQUEY_GRIPPER_MSGS::SETGRIPPERSTATE")
  (make-package "TUM_ICS_LACQUEY_GRIPPER_MSGS::SETGRIPPERSTATE"))
(unless (find-package "TUM_ICS_LACQUEY_GRIPPER_MSGS::SETGRIPPERSTATEREQUEST")
  (make-package "TUM_ICS_LACQUEY_GRIPPER_MSGS::SETGRIPPERSTATEREQUEST"))
(unless (find-package "TUM_ICS_LACQUEY_GRIPPER_MSGS::SETGRIPPERSTATERESPONSE")
  (make-package "TUM_ICS_LACQUEY_GRIPPER_MSGS::SETGRIPPERSTATERESPONSE"))

(in-package "ROS")





(defclass tum_ics_lacquey_gripper_msgs::setGripperStateRequest
  :super ros::object
  :slots (_newState ))

(defmethod tum_ics_lacquey_gripper_msgs::setGripperStateRequest
  (:init
   (&key
    ((:newState __newState) "")
    )
   (send-super :init)
   (setq _newState (string __newState))
   self)
  (:newState
   (&optional __newState)
   (if __newState (setq _newState __newState)) _newState)
  (:serialization-length
   ()
   (+
    ;; string _newState
    4 (length _newState)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _newState
       (write-long (length _newState) s) (princ _newState s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _newState
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _newState (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass tum_ics_lacquey_gripper_msgs::setGripperStateResponse
  :super ros::object
  :slots (_ok ))

(defmethod tum_ics_lacquey_gripper_msgs::setGripperStateResponse
  (:init
   (&key
    ((:ok __ok) nil)
    )
   (send-super :init)
   (setq _ok __ok)
   self)
  (:ok
   (&optional (__ok :null))
   (if (not (eq __ok :null)) (setq _ok __ok)) _ok)
  (:serialization-length
   ()
   (+
    ;; bool _ok
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _ok
       (if _ok (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _ok
     (setq _ok (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass tum_ics_lacquey_gripper_msgs::setGripperState
  :super ros::object
  :slots ())

(setf (get tum_ics_lacquey_gripper_msgs::setGripperState :md5sum-) "8223e23338c4551db84e86bb33c66825")
(setf (get tum_ics_lacquey_gripper_msgs::setGripperState :datatype-) "tum_ics_lacquey_gripper_msgs/setGripperState")
(setf (get tum_ics_lacquey_gripper_msgs::setGripperState :request) tum_ics_lacquey_gripper_msgs::setGripperStateRequest)
(setf (get tum_ics_lacquey_gripper_msgs::setGripperState :response) tum_ics_lacquey_gripper_msgs::setGripperStateResponse)

(defmethod tum_ics_lacquey_gripper_msgs::setGripperStateRequest
  (:response () (instance tum_ics_lacquey_gripper_msgs::setGripperStateResponse :init)))

(setf (get tum_ics_lacquey_gripper_msgs::setGripperStateRequest :md5sum-) "8223e23338c4551db84e86bb33c66825")
(setf (get tum_ics_lacquey_gripper_msgs::setGripperStateRequest :datatype-) "tum_ics_lacquey_gripper_msgs/setGripperStateRequest")
(setf (get tum_ics_lacquey_gripper_msgs::setGripperStateRequest :definition-)
      "# set gripper state
string newState
---
bool ok

")

(setf (get tum_ics_lacquey_gripper_msgs::setGripperStateResponse :md5sum-) "8223e23338c4551db84e86bb33c66825")
(setf (get tum_ics_lacquey_gripper_msgs::setGripperStateResponse :datatype-) "tum_ics_lacquey_gripper_msgs/setGripperStateResponse")
(setf (get tum_ics_lacquey_gripper_msgs::setGripperStateResponse :definition-)
      "# set gripper state
string newState
---
bool ok

")



(provide :tum_ics_lacquey_gripper_msgs/setGripperState "8223e23338c4551db84e86bb33c66825")


