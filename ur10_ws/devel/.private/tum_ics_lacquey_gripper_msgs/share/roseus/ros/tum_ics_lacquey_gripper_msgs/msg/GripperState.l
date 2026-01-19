;; Auto-generated. Do not edit!


(when (boundp 'tum_ics_lacquey_gripper_msgs::GripperState)
  (if (not (find-package "TUM_ICS_LACQUEY_GRIPPER_MSGS"))
    (make-package "TUM_ICS_LACQUEY_GRIPPER_MSGS"))
  (shadow 'GripperState (find-package "TUM_ICS_LACQUEY_GRIPPER_MSGS")))
(unless (find-package "TUM_ICS_LACQUEY_GRIPPER_MSGS::GRIPPERSTATE")
  (make-package "TUM_ICS_LACQUEY_GRIPPER_MSGS::GRIPPERSTATE"))

(in-package "ROS")
;;//! \htmlinclude GripperState.msg.html


(defclass tum_ics_lacquey_gripper_msgs::GripperState
  :super ros::object
  :slots (_state _stateId ))

(defmethod tum_ics_lacquey_gripper_msgs::GripperState
  (:init
   (&key
    ((:state __state) "")
    ((:stateId __stateId) 0)
    )
   (send-super :init)
   (setq _state (string __state))
   (setq _stateId (round __stateId))
   self)
  (:state
   (&optional __state)
   (if __state (setq _state __state)) _state)
  (:stateId
   (&optional __stateId)
   (if __stateId (setq _stateId __stateId)) _stateId)
  (:serialization-length
   ()
   (+
    ;; string _state
    4 (length _state)
    ;; int64 _stateId
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _state
       (write-long (length _state) s) (princ _state s)
     ;; int64 _stateId
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _stateId (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _stateId) (= (length (_stateId . bv)) 2)) ;; bignum
              (write-long (ash (elt (_stateId . bv) 0) 0) s)
              (write-long (ash (elt (_stateId . bv) 1) -1) s))
             ((and (class _stateId) (= (length (_stateId . bv)) 1)) ;; big1
              (write-long (elt (_stateId . bv) 0) s)
              (write-long (if (>= _stateId 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _stateId s)(write-long (if (>= _stateId 0) 0 #xffffffff) s)))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _state
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _state (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; int64 _stateId
#+(or :alpha :irix6 :x86_64)
      (setf _stateId (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _stateId (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;;
   self)
  )

(setf (get tum_ics_lacquey_gripper_msgs::GripperState :md5sum-) "6806ba1ebf38d757b7a04e49116d3b97")
(setf (get tum_ics_lacquey_gripper_msgs::GripperState :datatype-) "tum_ics_lacquey_gripper_msgs/GripperState")
(setf (get tum_ics_lacquey_gripper_msgs::GripperState :definition-)
      "# gripper state
string state
int64 stateId

")



(provide :tum_ics_lacquey_gripper_msgs/GripperState "6806ba1ebf38d757b7a04e49116d3b97")


