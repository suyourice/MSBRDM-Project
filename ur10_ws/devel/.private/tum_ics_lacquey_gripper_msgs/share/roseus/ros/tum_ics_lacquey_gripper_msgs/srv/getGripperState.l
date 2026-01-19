;; Auto-generated. Do not edit!


(when (boundp 'tum_ics_lacquey_gripper_msgs::getGripperState)
  (if (not (find-package "TUM_ICS_LACQUEY_GRIPPER_MSGS"))
    (make-package "TUM_ICS_LACQUEY_GRIPPER_MSGS"))
  (shadow 'getGripperState (find-package "TUM_ICS_LACQUEY_GRIPPER_MSGS")))
(unless (find-package "TUM_ICS_LACQUEY_GRIPPER_MSGS::GETGRIPPERSTATE")
  (make-package "TUM_ICS_LACQUEY_GRIPPER_MSGS::GETGRIPPERSTATE"))
(unless (find-package "TUM_ICS_LACQUEY_GRIPPER_MSGS::GETGRIPPERSTATEREQUEST")
  (make-package "TUM_ICS_LACQUEY_GRIPPER_MSGS::GETGRIPPERSTATEREQUEST"))
(unless (find-package "TUM_ICS_LACQUEY_GRIPPER_MSGS::GETGRIPPERSTATERESPONSE")
  (make-package "TUM_ICS_LACQUEY_GRIPPER_MSGS::GETGRIPPERSTATERESPONSE"))

(in-package "ROS")





(defclass tum_ics_lacquey_gripper_msgs::getGripperStateRequest
  :super ros::object
  :slots ())

(defmethod tum_ics_lacquey_gripper_msgs::getGripperStateRequest
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(defclass tum_ics_lacquey_gripper_msgs::getGripperStateResponse
  :super ros::object
  :slots (_states _currentState ))

(defmethod tum_ics_lacquey_gripper_msgs::getGripperStateResponse
  (:init
   (&key
    ((:states __states) (let (r) (dotimes (i 0) (push "" r)) r))
    ((:currentState __currentState) "")
    )
   (send-super :init)
   (setq _states __states)
   (setq _currentState (string __currentState))
   self)
  (:states
   (&optional __states)
   (if __states (setq _states __states)) _states)
  (:currentState
   (&optional __currentState)
   (if __currentState (setq _currentState __currentState)) _currentState)
  (:serialization-length
   ()
   (+
    ;; string[] _states
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _states)) 4
    ;; string _currentState
    4 (length _currentState)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string[] _states
     (write-long (length _states) s)
     (dolist (elem _states)
       (write-long (length elem) s) (princ elem s)
       )
     ;; string _currentState
       (write-long (length _currentState) s) (princ _currentState s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string[] _states
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _states (make-list n))
     (dotimes (i n)
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setf (elt _states i) (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
     ))
   ;; string _currentState
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _currentState (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass tum_ics_lacquey_gripper_msgs::getGripperState
  :super ros::object
  :slots ())

(setf (get tum_ics_lacquey_gripper_msgs::getGripperState :md5sum-) "ff0be24d823aa1cfde92641488f9e02a")
(setf (get tum_ics_lacquey_gripper_msgs::getGripperState :datatype-) "tum_ics_lacquey_gripper_msgs/getGripperState")
(setf (get tum_ics_lacquey_gripper_msgs::getGripperState :request) tum_ics_lacquey_gripper_msgs::getGripperStateRequest)
(setf (get tum_ics_lacquey_gripper_msgs::getGripperState :response) tum_ics_lacquey_gripper_msgs::getGripperStateResponse)

(defmethod tum_ics_lacquey_gripper_msgs::getGripperStateRequest
  (:response () (instance tum_ics_lacquey_gripper_msgs::getGripperStateResponse :init)))

(setf (get tum_ics_lacquey_gripper_msgs::getGripperStateRequest :md5sum-) "ff0be24d823aa1cfde92641488f9e02a")
(setf (get tum_ics_lacquey_gripper_msgs::getGripperStateRequest :datatype-) "tum_ics_lacquey_gripper_msgs/getGripperStateRequest")
(setf (get tum_ics_lacquey_gripper_msgs::getGripperStateRequest :definition-)
      "---
string[] states
string currentState

")

(setf (get tum_ics_lacquey_gripper_msgs::getGripperStateResponse :md5sum-) "ff0be24d823aa1cfde92641488f9e02a")
(setf (get tum_ics_lacquey_gripper_msgs::getGripperStateResponse :datatype-) "tum_ics_lacquey_gripper_msgs/getGripperStateResponse")
(setf (get tum_ics_lacquey_gripper_msgs::getGripperStateResponse :definition-)
      "---
string[] states
string currentState

")



(provide :tum_ics_lacquey_gripper_msgs/getGripperState "ff0be24d823aa1cfde92641488f9e02a")


