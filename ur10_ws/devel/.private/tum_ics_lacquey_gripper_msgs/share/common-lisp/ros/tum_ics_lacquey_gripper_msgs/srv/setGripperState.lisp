; Auto-generated. Do not edit!


(cl:in-package tum_ics_lacquey_gripper_msgs-srv)


;//! \htmlinclude setGripperState-request.msg.html

(cl:defclass <setGripperState-request> (roslisp-msg-protocol:ros-message)
  ((newState
    :reader newState
    :initarg :newState
    :type cl:string
    :initform ""))
)

(cl:defclass setGripperState-request (<setGripperState-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <setGripperState-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'setGripperState-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name tum_ics_lacquey_gripper_msgs-srv:<setGripperState-request> is deprecated: use tum_ics_lacquey_gripper_msgs-srv:setGripperState-request instead.")))

(cl:ensure-generic-function 'newState-val :lambda-list '(m))
(cl:defmethod newState-val ((m <setGripperState-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tum_ics_lacquey_gripper_msgs-srv:newState-val is deprecated.  Use tum_ics_lacquey_gripper_msgs-srv:newState instead.")
  (newState m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <setGripperState-request>) ostream)
  "Serializes a message object of type '<setGripperState-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'newState))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'newState))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <setGripperState-request>) istream)
  "Deserializes a message object of type '<setGripperState-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'newState) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'newState) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<setGripperState-request>)))
  "Returns string type for a service object of type '<setGripperState-request>"
  "tum_ics_lacquey_gripper_msgs/setGripperStateRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'setGripperState-request)))
  "Returns string type for a service object of type 'setGripperState-request"
  "tum_ics_lacquey_gripper_msgs/setGripperStateRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<setGripperState-request>)))
  "Returns md5sum for a message object of type '<setGripperState-request>"
  "8223e23338c4551db84e86bb33c66825")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'setGripperState-request)))
  "Returns md5sum for a message object of type 'setGripperState-request"
  "8223e23338c4551db84e86bb33c66825")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<setGripperState-request>)))
  "Returns full string definition for message of type '<setGripperState-request>"
  (cl:format cl:nil "# set gripper state~%string newState~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'setGripperState-request)))
  "Returns full string definition for message of type 'setGripperState-request"
  (cl:format cl:nil "# set gripper state~%string newState~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <setGripperState-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'newState))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <setGripperState-request>))
  "Converts a ROS message object to a list"
  (cl:list 'setGripperState-request
    (cl:cons ':newState (newState msg))
))
;//! \htmlinclude setGripperState-response.msg.html

(cl:defclass <setGripperState-response> (roslisp-msg-protocol:ros-message)
  ((ok
    :reader ok
    :initarg :ok
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass setGripperState-response (<setGripperState-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <setGripperState-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'setGripperState-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name tum_ics_lacquey_gripper_msgs-srv:<setGripperState-response> is deprecated: use tum_ics_lacquey_gripper_msgs-srv:setGripperState-response instead.")))

(cl:ensure-generic-function 'ok-val :lambda-list '(m))
(cl:defmethod ok-val ((m <setGripperState-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tum_ics_lacquey_gripper_msgs-srv:ok-val is deprecated.  Use tum_ics_lacquey_gripper_msgs-srv:ok instead.")
  (ok m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <setGripperState-response>) ostream)
  "Serializes a message object of type '<setGripperState-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'ok) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <setGripperState-response>) istream)
  "Deserializes a message object of type '<setGripperState-response>"
    (cl:setf (cl:slot-value msg 'ok) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<setGripperState-response>)))
  "Returns string type for a service object of type '<setGripperState-response>"
  "tum_ics_lacquey_gripper_msgs/setGripperStateResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'setGripperState-response)))
  "Returns string type for a service object of type 'setGripperState-response"
  "tum_ics_lacquey_gripper_msgs/setGripperStateResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<setGripperState-response>)))
  "Returns md5sum for a message object of type '<setGripperState-response>"
  "8223e23338c4551db84e86bb33c66825")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'setGripperState-response)))
  "Returns md5sum for a message object of type 'setGripperState-response"
  "8223e23338c4551db84e86bb33c66825")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<setGripperState-response>)))
  "Returns full string definition for message of type '<setGripperState-response>"
  (cl:format cl:nil "bool ok~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'setGripperState-response)))
  "Returns full string definition for message of type 'setGripperState-response"
  (cl:format cl:nil "bool ok~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <setGripperState-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <setGripperState-response>))
  "Converts a ROS message object to a list"
  (cl:list 'setGripperState-response
    (cl:cons ':ok (ok msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'setGripperState)))
  'setGripperState-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'setGripperState)))
  'setGripperState-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'setGripperState)))
  "Returns string type for a service object of type '<setGripperState>"
  "tum_ics_lacquey_gripper_msgs/setGripperState")