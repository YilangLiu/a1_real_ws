; Auto-generated. Do not edit!


(cl:in-package a1_msgs-msg)


;//! \htmlinclude FootState.msg.html

(cl:defclass <FootState> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (traj_index
    :reader traj_index
    :initarg :traj_index
    :type cl:integer
    :initform 0)
   (position
    :reader position
    :initarg :position
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (velocity
    :reader velocity
    :initarg :velocity
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (acceleration
    :reader acceleration
    :initarg :acceleration
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (contact
    :reader contact
    :initarg :contact
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass FootState (<FootState>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <FootState>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'FootState)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name a1_msgs-msg:<FootState> is deprecated: use a1_msgs-msg:FootState instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <FootState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader a1_msgs-msg:header-val is deprecated.  Use a1_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'traj_index-val :lambda-list '(m))
(cl:defmethod traj_index-val ((m <FootState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader a1_msgs-msg:traj_index-val is deprecated.  Use a1_msgs-msg:traj_index instead.")
  (traj_index m))

(cl:ensure-generic-function 'position-val :lambda-list '(m))
(cl:defmethod position-val ((m <FootState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader a1_msgs-msg:position-val is deprecated.  Use a1_msgs-msg:position instead.")
  (position m))

(cl:ensure-generic-function 'velocity-val :lambda-list '(m))
(cl:defmethod velocity-val ((m <FootState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader a1_msgs-msg:velocity-val is deprecated.  Use a1_msgs-msg:velocity instead.")
  (velocity m))

(cl:ensure-generic-function 'acceleration-val :lambda-list '(m))
(cl:defmethod acceleration-val ((m <FootState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader a1_msgs-msg:acceleration-val is deprecated.  Use a1_msgs-msg:acceleration instead.")
  (acceleration m))

(cl:ensure-generic-function 'contact-val :lambda-list '(m))
(cl:defmethod contact-val ((m <FootState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader a1_msgs-msg:contact-val is deprecated.  Use a1_msgs-msg:contact instead.")
  (contact m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <FootState>) ostream)
  "Serializes a message object of type '<FootState>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'traj_index)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'traj_index)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'traj_index)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'traj_index)) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'position) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'velocity) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'acceleration) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'contact) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <FootState>) istream)
  "Deserializes a message object of type '<FootState>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'traj_index)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'traj_index)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'traj_index)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'traj_index)) (cl:read-byte istream))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'position) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'velocity) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'acceleration) istream)
    (cl:setf (cl:slot-value msg 'contact) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<FootState>)))
  "Returns string type for a message object of type '<FootState>"
  "a1_msgs/FootState")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FootState)))
  "Returns string type for a message object of type 'FootState"
  "a1_msgs/FootState")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<FootState>)))
  "Returns md5sum for a message object of type '<FootState>"
  "91b05fb12025a7e2b535faab0de2315c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'FootState)))
  "Returns md5sum for a message object of type 'FootState"
  "91b05fb12025a7e2b535faab0de2315c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<FootState>)))
  "Returns full string definition for message of type '<FootState>"
  (cl:format cl:nil "# This is a message to hold the state of a single foot of a legged robot~%#~%# The states of each foot are stored in a vector of FootState messages~%# Accurate timing information is stored in the header~%~%Header header~%uint32 traj_index~%geometry_msgs/Vector3 position~%geometry_msgs/Vector3 velocity~%geometry_msgs/Vector3 acceleration~%bool contact~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'FootState)))
  "Returns full string definition for message of type 'FootState"
  (cl:format cl:nil "# This is a message to hold the state of a single foot of a legged robot~%#~%# The states of each foot are stored in a vector of FootState messages~%# Accurate timing information is stored in the header~%~%Header header~%uint32 traj_index~%geometry_msgs/Vector3 position~%geometry_msgs/Vector3 velocity~%geometry_msgs/Vector3 acceleration~%bool contact~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <FootState>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'position))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'velocity))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'acceleration))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <FootState>))
  "Converts a ROS message object to a list"
  (cl:list 'FootState
    (cl:cons ':header (header msg))
    (cl:cons ':traj_index (traj_index msg))
    (cl:cons ':position (position msg))
    (cl:cons ':velocity (velocity msg))
    (cl:cons ':acceleration (acceleration msg))
    (cl:cons ':contact (contact msg))
))
