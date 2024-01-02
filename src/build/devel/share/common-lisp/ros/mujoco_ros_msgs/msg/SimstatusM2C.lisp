; Auto-generated. Do not edit!


(cl:in-package mujoco_ros_msgs-msg)


;//! \htmlinclude SimstatusM2C.msg.html

(cl:defclass <SimstatusM2C> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (time
    :reader time
    :initarg :time
    :type cl:float
    :initform 0.0)
   (sim_paused
    :reader sim_paused
    :initarg :sim_paused
    :type cl:boolean
    :initform cl:nil)
   (sim_reset
    :reader sim_reset
    :initarg :sim_reset
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass SimstatusM2C (<SimstatusM2C>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SimstatusM2C>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SimstatusM2C)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mujoco_ros_msgs-msg:<SimstatusM2C> is deprecated: use mujoco_ros_msgs-msg:SimstatusM2C instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <SimstatusM2C>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mujoco_ros_msgs-msg:header-val is deprecated.  Use mujoco_ros_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'time-val :lambda-list '(m))
(cl:defmethod time-val ((m <SimstatusM2C>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mujoco_ros_msgs-msg:time-val is deprecated.  Use mujoco_ros_msgs-msg:time instead.")
  (time m))

(cl:ensure-generic-function 'sim_paused-val :lambda-list '(m))
(cl:defmethod sim_paused-val ((m <SimstatusM2C>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mujoco_ros_msgs-msg:sim_paused-val is deprecated.  Use mujoco_ros_msgs-msg:sim_paused instead.")
  (sim_paused m))

(cl:ensure-generic-function 'sim_reset-val :lambda-list '(m))
(cl:defmethod sim_reset-val ((m <SimstatusM2C>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mujoco_ros_msgs-msg:sim_reset-val is deprecated.  Use mujoco_ros_msgs-msg:sim_reset instead.")
  (sim_reset m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SimstatusM2C>) ostream)
  "Serializes a message object of type '<SimstatusM2C>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'time))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'sim_paused) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'sim_reset) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SimstatusM2C>) istream)
  "Deserializes a message object of type '<SimstatusM2C>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'time) (roslisp-utils:decode-double-float-bits bits)))
    (cl:setf (cl:slot-value msg 'sim_paused) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'sim_reset) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SimstatusM2C>)))
  "Returns string type for a message object of type '<SimstatusM2C>"
  "mujoco_ros_msgs/SimstatusM2C")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SimstatusM2C)))
  "Returns string type for a message object of type 'SimstatusM2C"
  "mujoco_ros_msgs/SimstatusM2C")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SimstatusM2C>)))
  "Returns md5sum for a message object of type '<SimstatusM2C>"
  "59140af914df0ac28c7df06d452568bd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SimstatusM2C)))
  "Returns md5sum for a message object of type 'SimstatusM2C"
  "59140af914df0ac28c7df06d452568bd")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SimstatusM2C>)))
  "Returns full string definition for message of type '<SimstatusM2C>"
  (cl:format cl:nil "#MUJOCO -> CONTROLLER SIM STATUS COMMANDER~%~%~%Header header~%float64 time~%~%bool sim_paused~%bool sim_reset~%~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SimstatusM2C)))
  "Returns full string definition for message of type 'SimstatusM2C"
  (cl:format cl:nil "#MUJOCO -> CONTROLLER SIM STATUS COMMANDER~%~%~%Header header~%float64 time~%~%bool sim_paused~%bool sim_reset~%~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SimstatusM2C>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     8
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SimstatusM2C>))
  "Converts a ROS message object to a list"
  (cl:list 'SimstatusM2C
    (cl:cons ':header (header msg))
    (cl:cons ':time (time msg))
    (cl:cons ':sim_paused (sim_paused msg))
    (cl:cons ':sim_reset (sim_reset msg))
))
