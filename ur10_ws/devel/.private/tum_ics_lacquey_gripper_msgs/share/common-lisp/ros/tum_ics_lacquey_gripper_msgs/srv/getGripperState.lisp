; Auto-generated. Do not edit!


(cl:in-package tum_ics_lacquey_gripper_msgs-srv)


;//! \htmlinclude getGripperState-request.msg.html

(cl:defclass <getGripperState-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass getGripperState-request (<getGripperState-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <getGripperState-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'getGripperState-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name tum_ics_lacquey_gripper_msgs-srv:<getGripperState-request> is deprecated: use tum_ics_lacquey_gripper_msgs-srv:getGripperState-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <getGripperState-request>) ostream)
  "Serializes a message object of type '<getGripperState-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <getGripperState-request>) istream)
  "Deserializes a message object of type '<getGripperState-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<getGripperState-request>)))
  "Returns string type for a service object of type '<getGripperState-request>"
  "tum_ics_lacquey_gripper_msgs/getGripperStateRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'getGripperState-request)))
  "Returns string type for a service object of type 'getGripperState-request"
  "tum_ics_lacquey_gripper_msgs/getGripperStateRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<getGripperState-request>)))
  "Returns md5sum for a message object of type '<getGripperState-request>"
  "ff0be24d823aa1cfde92641488f9e02a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'getGripperState-request)))
  "Returns md5sum for a message object of type 'getGripperState-request"
  "ff0be24d823aa1cfde92641488f9e02a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<getGripperState-request>)))
  "Returns full string definition for message of type '<getGripperState-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'getGripperState-request)))
  "Returns full string definition for message of type 'getGripperState-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <getGripperState-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <getGripperState-request>))
  "Converts a ROS message object to a list"
  (cl:list 'getGripperState-request
))
;//! \htmlinclude getGripperState-response.msg.html

(cl:defclass <getGripperState-response> (roslisp-msg-protocol:ros-message)
  ((states
    :reader states
    :initarg :states
    :type (cl:vector cl:string)
   :initform (cl:make-array 0 :element-type 'cl:string :initial-element ""))
   (currentState
    :reader currentState
    :initarg :currentState
    :type cl:string
    :initform ""))
)

(cl:defclass getGripperState-response (<getGripperState-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <getGripperState-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'getGripperState-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name tum_ics_lacquey_gripper_msgs-srv:<getGripperState-response> is deprecated: use tum_ics_lacquey_gripper_msgs-srv:getGripperState-response instead.")))

(cl:ensure-generic-function 'states-val :lambda-list '(m))
(cl:defmethod states-val ((m <getGripperState-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tum_ics_lacquey_gripper_msgs-srv:states-val is deprecated.  Use tum_ics_lacquey_gripper_msgs-srv:states instead.")
  (states m))

(cl:ensure-generic-function 'currentState-val :lambda-list '(m))
(cl:defmethod currentState-val ((m <getGripperState-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tum_ics_lacquey_gripper_msgs-srv:currentState-val is deprecated.  Use tum_ics_lacquey_gripper_msgs-srv:currentState instead.")
  (currentState m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <getGripperState-response>) ostream)
  "Serializes a message object of type '<getGripperState-response>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'states))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((__ros_str_len (cl:length ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) ele))
   (cl:slot-value msg 'states))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'currentState))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'currentState))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <getGripperState-response>) istream)
  "Deserializes a message object of type '<getGripperState-response>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'states) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'states)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:aref vals i) __ros_str_idx) (cl:code-char (cl:read-byte istream))))))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'currentState) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'currentState) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<getGripperState-response>)))
  "Returns string type for a service object of type '<getGripperState-response>"
  "tum_ics_lacquey_gripper_msgs/getGripperStateResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'getGripperState-response)))
  "Returns string type for a service object of type 'getGripperState-response"
  "tum_ics_lacquey_gripper_msgs/getGripperStateResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<getGripperState-response>)))
  "Returns md5sum for a message object of type '<getGripperState-response>"
  "ff0be24d823aa1cfde92641488f9e02a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'getGripperState-response)))
  "Returns md5sum for a message object of type 'getGripperState-response"
  "ff0be24d823aa1cfde92641488f9e02a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<getGripperState-response>)))
  "Returns full string definition for message of type '<getGripperState-response>"
  (cl:format cl:nil "string[] states~%string currentState~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'getGripperState-response)))
  "Returns full string definition for message of type 'getGripperState-response"
  (cl:format cl:nil "string[] states~%string currentState~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <getGripperState-response>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'states) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
     4 (cl:length (cl:slot-value msg 'currentState))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <getGripperState-response>))
  "Converts a ROS message object to a list"
  (cl:list 'getGripperState-response
    (cl:cons ':states (states msg))
    (cl:cons ':currentState (currentState msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'getGripperState)))
  'getGripperState-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'getGripperState)))
  'getGripperState-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'getGripperState)))
  "Returns string type for a service object of type '<getGripperState>"
  "tum_ics_lacquey_gripper_msgs/getGripperState")